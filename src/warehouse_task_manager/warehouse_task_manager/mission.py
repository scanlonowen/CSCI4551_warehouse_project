#!/usr/bin/env python3
"""
Stage 5.6 – Autonomous pallet pick-and-deliver mission.

Runs after the warehouse has been explored and the SLAM map is live.
Subclasses BasicNavigator to add pallet-following subscriptions and
service clients alongside the nav2_simple_commander machinery.

State sequence:
  WAIT_NAV2 → GO_TO_ENTRANCE → APPROACH_PALLET → ATTACH_PALLET
  → CHOOSE_DROPOFF → GO_TO_DROPOFF → DETACH_PALLET → DONE

Drop-off selection uses Nav2's ComputePathToPose action to measure actual
path lengths to each candidate, then picks the shortest.  Falls back to
Euclidean distance if the planner can't compute a path.

Run:
    ros2 run warehouse_task_manager mission
(normally started automatically by mission.launch.py)
"""

import math
import os
import time
from typing import Dict, Optional, Tuple

import rclpy
from rclpy.duration import Duration
from rclpy.parameter import Parameter
import yaml

from ament_index_python.packages import get_package_share_directory

from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path
from std_msgs.msg import Float32, Float64MultiArray
from std_srvs.srv import Trigger

import tf2_ros
from tf2_ros import TransformException

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


# ── Helpers ──────────────────────────────────────────────────────────────────

def _make_pose(stamp, x: float, y: float, yaw: float) -> PoseStamped:
    p = PoseStamped()
    p.header.frame_id = 'map'
    p.header.stamp = stamp
    p.pose.position.x = float(x)
    p.pose.position.y = float(y)
    p.pose.orientation.z = math.sin(yaw / 2.0)
    p.pose.orientation.w = math.cos(yaw / 2.0)
    return p


def _path_length(path: Path) -> float:
    total = 0.0
    poses = path.poses
    for i in range(1, len(poses)):
        dx = poses[i].pose.position.x - poses[i - 1].pose.position.x
        dy = poses[i].pose.position.y - poses[i - 1].pose.position.y
        total += math.hypot(dx, dy)
    return total


def _load_config() -> dict:
    pkg = get_package_share_directory('warehouse_robot_bringup')
    path = os.path.join(pkg, 'config', 'mission.yaml')
    with open(path) as f:
        return yaml.safe_load(f)


# ── Mission class ─────────────────────────────────────────────────────────────

class ForkliftMission(BasicNavigator):
    """
    Extends BasicNavigator with pallet perception subscriptions, actuator
    publishers, and the full pick-and-deliver state machine in run().
    """

    def __init__(self):
        super().__init__('forklift_mission')
        self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])

        config = _load_config()
        self._entrance: Dict = config['entrance_approach']
        self._dropoffs: Dict = config['dropoffs']

        # TF for current-pose lookups (used by dropoff selector)
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # Pallet perception state
        self._pallet_offset: Optional[float] = None
        self._pallet_pose: Optional[PoseStamped] = None

        self.create_subscription(Float32, '/pallet_offset', self._on_offset, 10)
        self.create_subscription(PoseStamped, '/pallet_pose', self._on_pallet_pose, 10)

        # Actuators
        self._cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self._fork_pub = self.create_publisher(
            Float64MultiArray, '/fork_position_controller/commands', 10
        )

        # Pallet attach / detach services
        self._attach_cli = self.create_client(Trigger, '/pallet/attach')
        self._detach_cli = self.create_client(Trigger, '/pallet/detach')

        # Metrics accumulated across the mission
        self._mission_start: float = 0.0
        self._total_nav_dist: float = 0.0

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _on_offset(self, msg: Float32):
        self._pallet_offset = msg.data

    def _on_pallet_pose(self, msg: PoseStamped):
        self._pallet_pose = msg

    # ── Utilities ─────────────────────────────────────────────────────────────

    def _stamp(self):
        return self.get_clock().now().to_msg()

    def _pose(self, x: float, y: float, yaw: float) -> PoseStamped:
        return _make_pose(self._stamp(), x, y, yaw)

    def _set_forks(self, height: float):
        msg = Float64MultiArray()
        msg.data = [height]
        self._fork_pub.publish(msg)

    def _stop(self):
        self._cmd_pub.publish(Twist())

    def _robot_pose_map(self) -> Optional[Tuple[float, float, float]]:
        """Return (x, y, yaw) of base_link in map frame, or None on TF failure."""
        try:
            tf = self._tf_buffer.lookup_transform(
                'map', 'base_link',
                rclpy.time.Time(),
                timeout=Duration(seconds=0.5),
            )
        except TransformException:
            return None
        t = tf.transform.translation
        q = tf.transform.rotation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        return (t.x, t.y, yaw)

    def _pallet_range(self) -> Optional[float]:
        """Straight-line distance from robot to detected pallet in map frame."""
        pose = self._pallet_pose
        robot = self._robot_pose_map()
        if pose is None or robot is None:
            return None
        dx = pose.pose.position.x - robot[0]
        dy = pose.pose.position.y - robot[1]
        return math.hypot(dx, dy)

    def _call_trigger(self, client, name: str, timeout_sec: float = 10.0) -> bool:
        if not client.wait_for_service(timeout_sec=timeout_sec):
            self.get_logger().error(f'{name} service not available')
            return False
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        if not future.done():
            self.get_logger().error(f'{name} timed out')
            return False
        resp = future.result()
        lvl = self.get_logger().info if resp.success else self.get_logger().warn
        lvl(f'{name}: {resp.message}')
        return resp.success

    # ── Navigation helper ─────────────────────────────────────────────────────

    def _navigate_to(self, x: float, y: float, yaw: float, label: str) -> bool:
        """Navigate to a map-frame pose. Returns True on SUCCEEDED."""
        before = self._robot_pose_map()
        goal = self._pose(x, y, yaw)
        self.get_logger().info(
            f'Navigating to {label} ({x:.2f}, {y:.2f}, yaw={math.degrees(yaw):.0f}°)...'
        )
        self.goToPose(goal)

        while not self.isTaskComplete():
            rclpy.spin_once(self, timeout_sec=0.1)

        result = self.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info(f'Reached {label}.')
            after = self._robot_pose_map()
            if before and after:
                self._total_nav_dist += math.hypot(
                    after[0] - before[0], after[1] - before[1]
                )
            return True

        msg = {
            TaskResult.CANCELED: f'Navigation to {label} was canceled.',
            TaskResult.FAILED: f'Navigation to {label} failed.',
        }.get(result, f'Navigation to {label} ended with result={result}.')
        self.get_logger().error(msg)
        return False

    # ── Pallet approach ───────────────────────────────────────────────────────

    def _approach_pallet(
        self,
        stop_range_m: float = 0.7,
        timeout_sec: float = 45.0,
        forward_speed: float = 0.08,
        turn_gain: float = 0.3,
        center_tol: float = 0.08,
    ) -> bool:
        """
        Drive toward the pallet using /pallet_offset for alignment.
        Stops when /pallet_pose distance < stop_range_m or after timeout_sec.
        Returns True if stopped within range, False if approach timed out.
        """
        self.get_logger().info('Approaching pallet via visual alignment...')
        deadline = time.time() + timeout_sec

        while rclpy.ok() and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

            rng = self._pallet_range()
            if rng is not None and rng < stop_range_m:
                self._stop()
                self.get_logger().info(f'Pallet in range ({rng:.2f} m) — stopping.')
                return True

            cmd = Twist()
            offset = self._pallet_offset

            if offset is None:
                cmd.angular.z = 0.2          # spin slowly to search
            elif abs(offset) > center_tol:
                cmd.angular.z = -turn_gain * offset   # align, don't advance
            else:
                cmd.linear.x = forward_speed          # centred — drive forward

            self._cmd_pub.publish(cmd)
            time.sleep(0.1)

        self._stop()
        self.get_logger().warn(
            f'Pallet approach timed out after {timeout_sec:.0f} s; '
            'proceeding with attach anyway.'
        )
        return False

    # ── Drop-off selection ────────────────────────────────────────────────────

    def _select_dropoff(self) -> Tuple[str, Dict, float]:
        """
        Return (name, waypoint_dict, cost_m) for the best drop-off candidate.

        Computes the Nav2 path length from the current robot pose to each
        candidate via computePathToPose.  Falls back to Euclidean distance
        if the planner fails for a candidate.
        """
        self.get_logger().info('Selecting drop-off bay by path length...')
        robot = self._robot_pose_map()

        costs: Dict[str, float] = {}
        for name, wp in self._dropoffs.items():
            goal = self._pose(wp['x'], wp['y'], wp.get('yaw', 0.0))
            cost = None

            try:
                start = self._pose(*robot) if robot else PoseStamped()
                path = self.computePathToPose(start, goal)
                if path and path.poses:
                    cost = _path_length(path)
                    self.get_logger().info(
                        f'  {name}: path = {cost:.2f} m'
                    )
            except Exception as exc:
                self.get_logger().warn(
                    f'  {name}: path computation failed ({exc}), using Euclidean'
                )

            if cost is None:
                if robot:
                    cost = math.hypot(wp['x'] - robot[0], wp['y'] - robot[1])
                    self.get_logger().info(f'  {name}: Euclidean = {cost:.2f} m')
                else:
                    cost = float('inf')

            costs[name] = cost

        best = min(costs, key=lambda n: costs[n])
        desc = self._dropoffs[best].get('description', best)
        self.get_logger().info(
            f'Selected: {best} ({desc}) — cost = {costs[best]:.2f} m'
        )
        return best, self._dropoffs[best], costs[best]

    # ── Main state machine ────────────────────────────────────────────────────

    def run(self):
        self._mission_start = time.time()

        # 1. Wait for Nav2
        self.get_logger().info('Mission node active — waiting for Nav2...')
        self.waitUntilNav2Active()
        self.get_logger().info('Nav2 active. Starting pick-and-deliver mission.')

        self._set_forks(0.0)   # forks down for pallet approach

        # 2. Navigate to entrance approach point
        e = self._entrance
        if not self._navigate_to(e['x'], e['y'], e['yaw'], 'entrance approach'):
            self.get_logger().error('Could not reach entrance approach — aborting.')
            return

        # 3. Visual approach toward pallet
        self._approach_pallet()
        self._stop()
        time.sleep(0.3)

        # 4. Attach pallet
        self.get_logger().info('Attaching pallet...')
        self._call_trigger(self._attach_cli, '/pallet/attach')
        time.sleep(0.5)

        # 5. Select drop-off
        dropoff_name, dropoff_wp, dropoff_cost = self._select_dropoff()

        # 6. Navigate to drop-off
        if not self._navigate_to(
            dropoff_wp['x'], dropoff_wp['y'], dropoff_wp.get('yaw', 0.0),
            f'dropoff {dropoff_name}',
        ):
            self.get_logger().error(
                f'Could not reach {dropoff_name} — detaching and aborting.'
            )
            self._call_trigger(self._detach_cli, '/pallet/detach')
            return

        # 7. Detach pallet
        self._stop()
        self.get_logger().info('Detaching pallet...')
        self._call_trigger(self._detach_cli, '/pallet/detach')

        # 8. Summary
        elapsed = time.time() - self._mission_start
        bar = '=' * 60
        desc = dropoff_wp.get('description', dropoff_name)
        self.get_logger().info(bar)
        self.get_logger().info('MISSION COMPLETE')
        self.get_logger().info(bar)
        self.get_logger().info(f'  Chosen drop-off  : {dropoff_name} — {desc}')
        self.get_logger().info(f'  Drop-off cost    : {dropoff_cost:.2f} m')
        self.get_logger().info(f'  Navigation dist  : {self._total_nav_dist:.2f} m (straight-line)')
        self.get_logger().info(f'  Total time       : {elapsed:.1f} s')
        self.get_logger().info(bar)


def main(args=None):
    rclpy.init(args=args)
    node = ForkliftMission()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node._stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
