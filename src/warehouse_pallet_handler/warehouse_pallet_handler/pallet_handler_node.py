#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# warehouse_pallet_handler / pallet_handler_node.py
#
# Why Option B (teleport tracker) instead of Option A (DetachableJoint)?
# ---------------------------------------------------------------------
# Gazebo Harmonic ships gz::sim::systems::DetachableJoint, but actually
# *creating* one at runtime requires either:
#   (a) injecting an SDF <joint> into a live world via gz transport (no public
#       ROS service for this in Jazzy's ros_gz_bridge), or
#   (b) pre-declaring the joint in the world/SDF and toggling it via a custom
#       topic — which still requires editing the URDF/world to add the
#       plugin and a child link to attach to.
# The brief explicitly forbids URDF edits and warns that the gz service path
# is fiddly. Option B is dramatically simpler to ship and debug visually:
# the AWS pallet model is <static>1</static>, so set_pose(name=...) works
# cleanly without fighting physics.
#
# gz service used (Harmonic):
#   /world/<world_name>/set_pose
#   --reqtype  gz.msgs.Pose
#   --reptype  gz.msgs.Boolean
# Confirmed by `gz service --list` semantics and the user_commands system
# (gz-sim-user-commands-system) loaded in the world SDF — that plugin is
# what advertises set_pose. If the user_commands plugin is *not* loaded the
# call will silently fail; we log and keep running.
#
# Note on world name: the brief assumes a `big_warehouse` world (Wave 1). At
# the time of writing this node, only `no_roof_small_warehouse` exists in
# this worktree, so the default `world_name` parameter matches that. Override
# with `--ros-args -p world_name:=big_warehouse` once Wave 1 lands.

import math
import shutil
import subprocess
import threading

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from std_srvs.srv import Trigger

from tf2_ros import Buffer, TransformListener, TransformException


# Fork-tip offset in base_link, taken from forklift.urdf.xacro:
#   chassis_bottom_length = 1.59
#   fork_length           = 0.99
#   fork_height           = 0.045
#   fork_base_height_offset = height_of_chassis_from_ground_plane - 0.076
#                           = 0.122 - 0.076 = 0.046
# Per the brief:
#   x = chassis_bottom_length/2 + fork_length/2 = 0.795 + 0.495 = 1.290
#   z = fork_base_height_offset + fork_height/2 = 0.046 + 0.0225 = 0.0685
# Note: the prismatic fork lift is currently treated as 0 (resting). A
# follow-up could read /joint_states and add the lift offset; for the
# attach mechanic that's overkill — we just want the pallet stuck to the
# carriage.
FORK_TIP_OFFSET_X = 1.59 / 2.0 + 0.99 / 2.0
FORK_TIP_OFFSET_Y = 0.0
FORK_TIP_OFFSET_Z = (0.122 - 0.076) + 0.045 / 2.0


def quat_to_yaw(qx, qy, qz, qw):
    """Yaw (Z rotation) from quaternion."""
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def yaw_to_quat(yaw):
    """Return (x, y, z, w) for a yaw-only rotation."""
    half = yaw * 0.5
    return (0.0, 0.0, math.sin(half), math.cos(half))


class PalletHandler(Node):

    def __init__(self):
        super().__init__('pallet_handler')

        # Parameters --------------------------------------------------
        self.declare_parameter('pallet_name', 'entrance_pallet')
        self.declare_parameter('world_name', 'big_warehouse')
        # World frame: 'auto' tries 'map' first, then falls back to 'odom'
        # so the handler works whether SLAM/Nav2 is running or not.
        # Set to a literal frame name (e.g. 'map' or 'odom') to force it.
        self.declare_parameter('world_frame', 'auto')
        self.declare_parameter('robot_frame', 'base_link')
        self.declare_parameter('rate_hz', 30.0)
        # Spawn pose of the forklift in the Gazebo world frame. Must match
        # warehouse_simulation.launch.py's spawn_x/spawn_y/spawn_z/spawn_yaw
        # (defaults are big_warehouse's entrance bay). The gz set_pose
        # service expects positions in the Gazebo world frame; ROS 'odom'
        # is rooted at the robot's spawn pose with yaw=0, so we have to
        # re-base every TF lookup through this offset.
        self.declare_parameter('spawn_x', 0.0)
        self.declare_parameter('spawn_y', -16.0)
        self.declare_parameter('spawn_z', 0.1)
        self.declare_parameter('spawn_yaw', 1.5708)

        self.pallet_name = self.get_parameter('pallet_name').value
        self.world_name = self.get_parameter('world_name').value
        self.world_frame = self.get_parameter('world_frame').value
        self.robot_frame = self.get_parameter('robot_frame').value
        rate_hz = float(self.get_parameter('rate_hz').value)
        self.spawn_x = float(self.get_parameter('spawn_x').value)
        self.spawn_y = float(self.get_parameter('spawn_y').value)
        self.spawn_z = float(self.get_parameter('spawn_z').value)
        self.spawn_yaw = float(self.get_parameter('spawn_yaw').value)

        # TF ----------------------------------------------------------
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # State -------------------------------------------------------
        self._attached = False
        self._lock = threading.Lock()

        # Resolve gz binary once. If the binary is missing we still
        # advertise the services so unit tests can run; we'll just log
        # an error on each tick.
        self._gz_bin = shutil.which('gz')
        if self._gz_bin is None:
            self.get_logger().warning(
                "'gz' CLI not on PATH; set_pose calls will be skipped.")

        self._set_pose_service = (
            f'/world/{self.world_name}/set_pose')

        # Services ----------------------------------------------------
        self.attach_srv = self.create_service(
            Trigger, '/pallet/attach', self._handle_attach)
        self.detach_srv = self.create_service(
            Trigger, '/pallet/detach', self._handle_detach)

        # 30 Hz timer. We always run the timer; it no-ops when not
        # attached. This keeps the control flow trivial.
        period = 1.0 / max(rate_hz, 1.0)
        self._timer = self.create_timer(period, self._tick)

        self.get_logger().info(
            f"pallet_handler ready (pallet='{self.pallet_name}', "
            f"world='{self.world_name}', service='{self._set_pose_service}')")

    # ----------------------------------------------------------------
    # Service handlers
    # ----------------------------------------------------------------
    def _resolve_world_frame(self):
        """Pick a usable world frame at runtime.

        If world_frame is 'auto', try 'map' first (stable when SLAM/AMCL is
        up), then fall back to 'odom' (always present once controllers run).
        Caches the resolved frame on success.
        """
        if self.world_frame != 'auto':
            return self.world_frame
        for candidate in ('map', 'odom'):
            try:
                self.tf_buffer.lookup_transform(
                    candidate, self.robot_frame,
                    rclpy.time.Time(),
                    timeout=Duration(seconds=0.2))
                if candidate == 'odom':
                    self.get_logger().warning(
                        "No 'map' frame; using 'odom'. Pallet pose will "
                        "drift if odom drifts. Run SLAM/AMCL for stability.")
                self.world_frame = candidate
                return candidate
            except TransformException:
                continue
        return None

    def _handle_attach(self, request, response):
        with self._lock:
            frame = self._resolve_world_frame()
            if frame is None:
                response.success = False
                response.message = (
                    f"No usable world frame ('map' or 'odom') -> "
                    f"{self.robot_frame}. Is the sim running?")
                self.get_logger().warning(response.message)
                return response

            self._attached = True
            response.success = True
            response.message = (
                f"Attached pallet '{self.pallet_name}' to fork tip "
                f"(world_frame='{frame}').")
            self.get_logger().info(response.message)
            return response

    def _handle_detach(self, request, response):
        with self._lock:
            was_attached = self._attached
            self._attached = False
        response.success = True
        response.message = (
            "Detached." if was_attached else "Already detached (no-op).")
        self.get_logger().info(response.message)
        return response

    # ----------------------------------------------------------------
    # Timer tick — drive pallet to fork tip.
    # ----------------------------------------------------------------
    def _tick(self):
        with self._lock:
            if not self._attached:
                return

        try:
            tf = self.tf_buffer.lookup_transform(
                self.world_frame, self.robot_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.05))
        except TransformException as ex:
            self.get_logger().warning(
                f"TF lookup failed during tick: {ex}", throttle_duration_sec=2.0)
            return

        # tf is world_frame -> base_link, where world_frame is 'odom' or
        # 'map'. ROS 'odom' is rooted at the robot's spawn pose with
        # yaw=0; the Gazebo world is rooted at the SDF origin. So we
        # compose: gz_world -> odom (via spawn pose) -> base_link (via tf)
        # -> fork_tip (via FORK_TIP_OFFSET). 'map' tracks 'odom' closely
        # enough that we treat them the same here; if drift becomes a
        # problem later, subscribe to /world/<name>/pose/info instead.
        t = tf.transform.translation
        q = tf.transform.rotation
        odom_yaw = quat_to_yaw(q.x, q.y, q.z, q.w)

        # base_link in the Gazebo world frame.
        cs, ss = math.cos(self.spawn_yaw), math.sin(self.spawn_yaw)
        bx = self.spawn_x + cs * t.x - ss * t.y
        by = self.spawn_y + ss * t.x + cs * t.y
        bz = self.spawn_z + t.z
        b_yaw = self.spawn_yaw + odom_yaw

        # Fork tip in the Gazebo world frame.
        cb, sb = math.cos(b_yaw), math.sin(b_yaw)
        wx = bx + cb * FORK_TIP_OFFSET_X - sb * FORK_TIP_OFFSET_Y
        wy = by + sb * FORK_TIP_OFFSET_X + cb * FORK_TIP_OFFSET_Y
        wz = bz + FORK_TIP_OFFSET_Z

        ox, oy, oz, ow = yaw_to_quat(b_yaw)
        self._send_set_pose(wx, wy, wz, ox, oy, oz, ow)

    # ----------------------------------------------------------------
    # gz transport via the gz CLI. Pragmatic shortcut: the python-gz
    # bindings aren't in Jazzy's apt set, and shelling out keeps the
    # dep surface tiny.
    # ----------------------------------------------------------------
    def _send_set_pose(self, x, y, z, qx, qy, qz, qw):
        if self._gz_bin is None:
            return

        req = (
            f'name: "{self.pallet_name}", '
            f'position: {{x: {x:.6f}, y: {y:.6f}, z: {z:.6f}}}, '
            f'orientation: {{x: {qx:.6f}, y: {qy:.6f}, '
            f'z: {qz:.6f}, w: {qw:.6f}}}')

        cmd = [
            self._gz_bin, 'service',
            '-s', self._set_pose_service,
            '--reqtype', 'gz.msgs.Pose',
            '--reptype', 'gz.msgs.Boolean',
            '--timeout', '200',
            '--req', req,
        ]
        try:
            result = subprocess.run(
                cmd, capture_output=True, text=True, timeout=0.5)
            if result.returncode != 0:
                self.get_logger().error(
                    f"gz set_pose failed (rc={result.returncode}): "
                    f"{result.stderr.strip()}",
                    throttle_duration_sec=2.0)
        except subprocess.TimeoutExpired:
            self.get_logger().warning(
                "gz set_pose timed out", throttle_duration_sec=2.0)
        except Exception as ex:  # noqa: BLE001 — keep node alive.
            self.get_logger().error(
                f"gz set_pose raised {ex!r}", throttle_duration_sec=2.0)


def main(argv=None):
    rclpy.init(args=argv)
    node = PalletHandler()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
