#!/usr/bin/env python3
"""
exploration_map_saver.py

One-shot helper that listens for explore_lite's "exploration complete" signal
and asks slam_toolbox to dump the live map to disk.

What it does
------------
1. Subscribes to /explore/status (explore_lite_msgs/ExploreStatus).
2. Subscribes to /diff_cont/odom to track how far the robot has moved.
3. When status == EXPLORATION_COMPLETE (or RETURNED_TO_ORIGIN) AND both of
   the following guards pass:
     - At least MIN_ELAPSED_SECS seconds since node startup (so a spurious
       COMPLETE arriving before exploration really begins is ignored), and
     - The robot has moved at least MIN_TRAVEL_METERS from its starting pose
       (so an instant COMPLETE due to "no frontiers found" is ignored),
   it logs a prominent "EXPLORATION COMPLETE" banner.
4. Calls /slam_toolbox/save_map with the requested map basename.
5. Calls /slam_toolbox/serialize_map with the same basename.
6. Exits cleanly. The launch is configured (on_exit=Shutdown) to tear down
   the rest of the stack when this node exits.

Usage
-----
    ros2 run warehouse_robot_bringup exploration_map_saver.py \
        --ros-args -p map_basename:=/abs/path/to/warehouse_explored

This script is launched automatically by explore.launch.py.
"""
import math
import os
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

from nav_msgs.msg import Odometry
from std_msgs.msg import String as StdString

try:
    from explore_lite_msgs.msg import ExploreStatus
except ImportError as e:
    print(f"[exploration_map_saver] explore_lite_msgs not found: {e}",
          file=sys.stderr)
    raise

try:
    from slam_toolbox.srv import SaveMap, SerializePoseGraph
except ImportError as e:
    print(f"[exploration_map_saver] slam_toolbox srvs not found: {e}",
          file=sys.stderr)
    raise

# Guards against false EXPLORATION_COMPLETE signals that arrive when
# explore_lite can't find frontiers at startup (e.g. costmap not ready).
MIN_ELAPSED_SECS = 30.0   # ignore COMPLETE within first 30 s of node startup
MIN_TRAVEL_METERS = 0.5   # ignore COMPLETE if robot hasn't moved ≥ 0.5 m


class ExplorationMapSaver(Node):
    def __init__(self):
        super().__init__('exploration_map_saver')

        self.declare_parameter(
            'map_basename',
            os.path.expanduser('~/warehouse_explored'),
        )
        self.declare_parameter('shutdown_on_complete', True)

        self.map_basename = self.get_parameter('map_basename') \
            .get_parameter_value().string_value
        self.shutdown_on_complete = self.get_parameter('shutdown_on_complete') \
            .get_parameter_value().bool_value

        parent = os.path.dirname(os.path.abspath(self.map_basename))
        if parent:
            os.makedirs(parent, exist_ok=True)

        self._start_time = time.monotonic()
        self._start_x: float | None = None
        self._start_y: float | None = None
        self._travel_m: float = 0.0
        self._already_saved = False
        self._save_pending = False
        self._save_done = False

        qos_tl = QoSProfile(depth=10)
        qos_tl.reliability = QoSReliabilityPolicy.RELIABLE
        qos_tl.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL

        self.sub = self.create_subscription(
            ExploreStatus, '/explore/status', self._on_status, qos_tl,
        )

        self.odom_sub = self.create_subscription(
            Odometry, '/diff_cont/odom', self._on_odom, 10,
        )

        self.save_cli = self.create_client(SaveMap, '/slam_toolbox/save_map')
        self.serialize_cli = self.create_client(
            SerializePoseGraph, '/slam_toolbox/serialize_map',
        )

        self.get_logger().info(
            f"exploration_map_saver ready; map_basename={self.map_basename}"
        )
        self.get_logger().info(
            f"Guards: ignore COMPLETE if elapsed < {MIN_ELAPSED_SECS} s "
            f"or travel < {MIN_TRAVEL_METERS} m"
        )
        self.get_logger().info(
            "Waiting for /explore/status EXPLORATION_COMPLETE ..."
        )

    def _on_odom(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        if self._start_x is None:
            self._start_x = x
            self._start_y = y
        dx = x - self._start_x
        dy = y - self._start_y
        self._travel_m = math.hypot(dx, dy)

    def _on_status(self, msg: ExploreStatus):
        if msg.status not in (
            ExploreStatus.EXPLORATION_COMPLETE,
            ExploreStatus.RETURNED_TO_ORIGIN,
        ):
            return
        if self._already_saved:
            return

        elapsed = time.monotonic() - self._start_time

        if elapsed < MIN_ELAPSED_SECS:
            self.get_logger().warn(
                f"Received EXPLORATION_COMPLETE after only {elapsed:.1f} s "
                f"(< {MIN_ELAPSED_SECS} s guard) — ignoring spurious completion. "
                f"Travel so far: {self._travel_m:.2f} m."
            )
            return

        if self._travel_m < MIN_TRAVEL_METERS:
            self.get_logger().warn(
                f"Received EXPLORATION_COMPLETE but robot only moved "
                f"{self._travel_m:.2f} m (< {MIN_TRAVEL_METERS} m guard) — "
                f"ignoring spurious completion."
            )
            return

        self.get_logger().info(
            f"Accepted EXPLORATION_COMPLETE (elapsed={elapsed:.1f} s, "
            f"travel={self._travel_m:.2f} m)."
        )
        self._already_saved = True
        self._save_pending = True

    @property
    def save_pending(self) -> bool:
        return getattr(self, '_save_pending', False)

    def _save_everything(self):
        bar = "=" * 60
        self.get_logger().info(bar)
        self.get_logger().info("EXPLORATION COMPLETE")
        self.get_logger().info(f"Saving map to: {self.map_basename}")
        self.get_logger().info(bar)

        ok_save = self._call_save_map()
        ok_serial = self._call_serialize_map()

        if ok_save and ok_serial:
            self.get_logger().info(
                f"Map artefacts written:\n"
                f"  {self.map_basename}.yaml\n"
                f"  {self.map_basename}.pgm\n"
                f"  {self.map_basename}.posegraph\n"
                f"  {self.map_basename}.data"
            )
        else:
            self.get_logger().error(
                "One or more slam_toolbox save calls failed; "
                "see logs above."
            )

        self._save_done = True

    def _call_save_map(self) -> bool:
        if not self.save_cli.wait_for_service(timeout_sec=10.0):
            self.get_logger().error("/slam_toolbox/save_map not available")
            return False
        req = SaveMap.Request()
        req.name = StdString(data=self.map_basename)
        future = self.save_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)
        if not future.done():
            self.get_logger().error("/slam_toolbox/save_map timed out")
            return False
        result = future.result()
        ok = getattr(result, 'result', 0) == 0
        self.get_logger().info(f"save_map returned: {result}")
        return ok

    def _call_serialize_map(self) -> bool:
        if not self.serialize_cli.wait_for_service(timeout_sec=10.0):
            self.get_logger().error("/slam_toolbox/serialize_map not available")
            return False
        req = SerializePoseGraph.Request()
        req.filename = self.map_basename
        future = self.serialize_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)
        if not future.done():
            self.get_logger().error("/slam_toolbox/serialize_map timed out")
            return False
        result = future.result()
        ok = getattr(result, 'result', 0) == 0
        self.get_logger().info(f"serialize_map returned: {result}")
        return ok


def main():
    rclpy.init()
    node = ExplorationMapSaver()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.2)
            if node.save_pending and not node._save_done:
                node._save_everything()
            if node._save_done:
                if node.shutdown_on_complete:
                    time.sleep(0.5)
                    node.get_logger().info(
                        "Map saved; exiting (launch will tear down).")
                    break
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
