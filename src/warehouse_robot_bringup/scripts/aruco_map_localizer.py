#!/usr/bin/env python3
"""
Camera-based global localization using fixed ArUco markers in the warehouse.

The node detects markers in /camera/image_raw, solves for the camera pose in
the map frame using known marker corner coordinates, and publishes a planar
map -> odom transform. With stable local odometry, that gives Nav2 a global
frame without relying on AMCL / lidar scan matching.
"""

import math
from pathlib import Path

import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener
import yaml


def quaternion_to_matrix(x, y, z, w):
    n = x * x + y * y + z * z + w * w
    if n < 1e-12:
        return np.eye(3)
    s = 2.0 / n
    xx, yy, zz = x * x * s, y * y * s, z * z * s
    xy, xz, yz = x * y * s, x * z * s, y * z * s
    wx, wy, wz = w * x * s, w * y * s, w * z * s
    return np.array([
        [1.0 - (yy + zz), xy - wz, xz + wy],
        [xy + wz, 1.0 - (xx + zz), yz - wx],
        [xz - wy, yz + wx, 1.0 - (xx + yy)],
    ])


def transform_to_matrix(transform) -> np.ndarray:
    t = transform.translation
    q = transform.rotation
    mat = np.eye(4)
    mat[:3, :3] = quaternion_to_matrix(q.x, q.y, q.z, q.w)
    mat[:3, 3] = [t.x, t.y, t.z]
    return mat


def xy_yaw_from_matrix(mat: np.ndarray):
    return (
        float(mat[0, 3]),
        float(mat[1, 3]),
        float(math.atan2(mat[1, 0], mat[0, 0])),
    )


def quaternion_from_yaw(yaw: float):
    half = yaw * 0.5
    return (0.0, 0.0, math.sin(half), math.cos(half))


def angle_diff(a: float, b: float) -> float:
    return math.atan2(math.sin(a - b), math.cos(a - b))


class ArucoMapLocalizer(Node):
    def __init__(self):
        super().__init__('aruco_map_localizer')

        if self.has_parameter('use_sim_time'):
            self.use_sim_time = bool(self.get_parameter('use_sim_time').value)
        else:
            self.use_sim_time = bool(self.declare_parameter('use_sim_time', False).value)
        default_config = str(
            Path(__file__).resolve().parents[1] / 'config' / 'fiducial_markers_big_warehouse.yaml'
        )
        self.marker_config = self.declare_parameter(
            'marker_config', default_config
        ).value
        self.image_topic = self.declare_parameter(
            'image_topic', '/camera/image_raw'
        ).value
        self.camera_info_topic = self.declare_parameter(
            'camera_info_topic', '/camera/camera_info'
        ).value
        self.map_frame = self.declare_parameter('map_frame', 'map').value
        self.odom_frame = self.declare_parameter('odom_frame', 'odom').value
        self.base_frame = self.declare_parameter('base_frame', 'drive_base_link').value
        self.publish_rate_hz = float(self.declare_parameter('publish_rate_hz', 10.0).value)
        self.smoothing_alpha = float(self.declare_parameter('smoothing_alpha', 0.55).value)
        self.map_x_offset = float(self.declare_parameter('map_x_offset', 0.0).value)
        self.map_y_offset = float(self.declare_parameter('map_y_offset', 0.0).value)
        self.map_yaw_offset = float(self.declare_parameter('map_yaw_offset', 0.0).value)
        self.lock_after_localization = bool(
            self.declare_parameter('lock_after_localization', True).value
        )
        self.navigate_to_pose_status_topic = self.declare_parameter(
            'navigate_to_pose_status_topic', '/navigate_to_pose/_action/status'
        ).value
        self.navigate_through_poses_status_topic = self.declare_parameter(
            'navigate_through_poses_status_topic', '/navigate_through_poses/_action/status'
        ).value
        self.motion_odom_topic = self.declare_parameter(
            'motion_odom_topic', '/ground_truth/odom'
        ).value
        self.lock_while_moving = bool(self.declare_parameter('lock_while_moving', True).value)
        self.stationary_hold_time = float(
            self.declare_parameter('stationary_hold_time', 1.0).value
        )
        self.linear_motion_threshold = float(
            self.declare_parameter('linear_motion_threshold', 0.03).value
        )
        self.angular_motion_threshold = float(
            self.declare_parameter('angular_motion_threshold', 0.08).value
        )
        self.max_correction_translation = float(
            self.declare_parameter('max_correction_translation', 0.30).value
        )
        self.max_correction_yaw = float(
            self.declare_parameter('max_correction_yaw', 0.35).value
        )
        self.ransac_reproj_error = float(
            self.declare_parameter('ransac_reproj_error', 3.0).value
        )

        self.bridge = CvBridge()
        self.tf_buffer = Buffer(cache_time=Duration(seconds=20.0))
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.camera_matrix = None
        self.dist_coeffs = None
        self.camera_frame = None
        self.last_map_to_odom = None
        self.last_publish_stamp = None
        self._last_detection_count = 0
        self.last_motion_time = None
        self.current_linear_speed = 0.0
        self.current_angular_speed = 0.0
        self._skipped_for_motion_count = 0
        self._rejected_jump_count = 0
        self.localization_locked = False
        self.goal_active = False

        self.marker_size, self.marker_points, self.dictionary_name = self._load_marker_config(
            self.marker_config
        )
        self.aruco_dict = getattr(cv2.aruco, self.dictionary_name)
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(self.aruco_dict)
        try:
            self.detector_params = cv2.aruco.DetectorParameters_create()
        except AttributeError:
            self.detector_params = cv2.aruco.DetectorParameters()

        sub_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        info_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.create_subscription(CameraInfo, self.camera_info_topic, self._camera_info_cb, info_qos)
        self.create_subscription(Image, self.image_topic, self._image_cb, sub_qos)
        self.create_subscription(Odometry, self.motion_odom_topic, self._odom_cb, 10)
        self.create_subscription(
            GoalStatusArray,
            self.navigate_to_pose_status_topic,
            self._goal_status_cb,
            10,
        )
        self.create_subscription(
            GoalStatusArray,
            self.navigate_through_poses_status_topic,
            self._goal_status_cb,
            10,
        )
        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/visual_global_pose', 10
        )
        self.create_timer(1.0 / max(self.publish_rate_hz, 1.0), self._republish_last_tf)

        self.get_logger().info(
            f'aruco_map_localizer ready: {len(self.marker_points)} markers from '
            f'{self.marker_config}, image={self.image_topic}, '
            f'camera_info={self.camera_info_topic}, motion_odom={self.motion_odom_topic}, '
            f'lock_after_localization={self.lock_after_localization}, '
            f'use_sim_time={self.use_sim_time}'
        )

    def _now_msg(self):
        return self.get_clock().now().to_msg()

    def _load_marker_config(self, path):
        with open(path, 'r', encoding='utf-8') as f:
            data = yaml.safe_load(f)

        marker_size = float(data['marker_size'])
        dictionary = data.get('dictionary', 'DICT_4X4_50')
        markers = {}
        for key, value in data['markers'].items():
            marker_id = int(key)
            center = np.array(value['center'], dtype=np.float32)
            normal_yaw = float(value['normal_yaw'])
            markers[marker_id] = self._corners_from_center(center, normal_yaw, marker_size)
        return marker_size, markers, dictionary

    @staticmethod
    def _corners_from_center(center, normal_yaw, size):
        normal = np.array([math.cos(normal_yaw), math.sin(normal_yaw), 0.0], dtype=np.float32)
        up = np.array([0.0, 0.0, 1.0], dtype=np.float32)
        right = np.cross(up, normal)
        right = right / np.linalg.norm(right)
        half = size / 2.0
        top_left = center - right * half + up * half
        top_right = center + right * half + up * half
        bottom_right = center + right * half - up * half
        bottom_left = center - right * half - up * half
        return np.array([top_left, top_right, bottom_right, bottom_left], dtype=np.float32)

    def _camera_info_cb(self, msg: CameraInfo):
        self.camera_matrix = np.array(msg.k, dtype=np.float64).reshape((3, 3))
        self.dist_coeffs = np.array(msg.d, dtype=np.float64) if msg.d else np.zeros((5,), dtype=np.float64)
        self.camera_frame = msg.header.frame_id or self.camera_frame

    def _odom_cb(self, msg: Odometry):
        linear = msg.twist.twist.linear
        angular = msg.twist.twist.angular
        self.current_linear_speed = float(math.hypot(linear.x, linear.y))
        self.current_angular_speed = float(abs(angular.z))
        if (
            self.current_linear_speed > self.linear_motion_threshold
            or self.current_angular_speed > self.angular_motion_threshold
        ):
            self.last_motion_time = self.get_clock().now()

    def _goal_status_cb(self, msg: GoalStatusArray):
        active_states = {
            GoalStatus.STATUS_ACCEPTED,
            GoalStatus.STATUS_EXECUTING,
            GoalStatus.STATUS_CANCELING,
        }
        terminal_states = {
            GoalStatus.STATUS_SUCCEEDED,
            GoalStatus.STATUS_CANCELED,
            GoalStatus.STATUS_ABORTED,
        }
        has_active = any(status.status in active_states for status in msg.status_list)
        has_terminal = any(status.status in terminal_states for status in msg.status_list)

        if has_active:
            if not self.goal_active:
                self.goal_active = True
                if self.lock_after_localization and self.last_map_to_odom is not None:
                    self.localization_locked = True
                    self.get_logger().info(
                        'aruco_map_localizer: navigation goal active, freezing tag relocalization until goal completion'
                    )
            return

        if self.goal_active and has_terminal:
            self.goal_active = False
            if self.lock_after_localization:
                self.localization_locked = False
                self.get_logger().info(
                    'aruco_map_localizer: navigation goal completed, unlocking for the next tag-based relocalization'
                )

    def _recently_moving(self) -> bool:
        if (
            not self.lock_while_moving
            or self.last_map_to_odom is None
            or not (self.goal_active or self.localization_locked)
        ):
            return False
        if self.last_motion_time is None:
            return False
        dt = (self.get_clock().now() - self.last_motion_time).nanoseconds / 1e9
        return dt < self.stationary_hold_time

    def _candidate_jump_is_reasonable(self, x: float, y: float, yaw: float) -> bool:
        if self.last_map_to_odom is None:
            return True
        last_x, last_y, last_yaw = self.last_map_to_odom
        translation_jump = math.hypot(x - last_x, y - last_y)
        yaw_jump = abs(angle_diff(yaw, last_yaw))
        return (
            translation_jump <= self.max_correction_translation
            and yaw_jump <= self.max_correction_yaw
        )

    def _lookup_transform_matrix(self, target_frame: str, source_frame: str):
        transform = self.tf_buffer.lookup_transform(
            target_frame,
            source_frame,
            Time(),
        )
        return transform_to_matrix(transform.transform)

    def _map_to_odom_from_pnp(self, rvec, tvec, camera_T_base, odom_T_base):
        cam_R_map, _ = cv2.Rodrigues(rvec)
        cam_T_map = np.eye(4)
        cam_T_map[:3, :3] = cam_R_map
        cam_T_map[:3, 3] = tvec.reshape(3)
        map_T_camera = np.linalg.inv(cam_T_map)
        map_T_base = map_T_camera @ camera_T_base
        map_T_odom = map_T_base @ np.linalg.inv(odom_T_base)
        x, y, yaw = xy_yaw_from_matrix(map_T_odom)
        return x, y, yaw

    def _image_cb(self, msg: Image):
        if self.camera_matrix is None:
            return

        camera_frame = msg.header.frame_id or self.camera_frame
        if not camera_frame:
            return

        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().warn(f'aruco_map_localizer: cv_bridge failed: {exc}')
            return

        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(
            gray,
            self.aruco_dict,
            parameters=self.detector_params,
        )
        if ids is None or len(ids) == 0:
            return

        try:
            camera_T_base = self._lookup_transform_matrix(camera_frame, self.base_frame)
            odom_T_base = self._lookup_transform_matrix(self.odom_frame, self.base_frame)
        except TransformException as exc:
            self.get_logger().warn(f'aruco_map_localizer: waiting for TF: {exc}')
            return

        all_object_points = []
        all_image_points = []
        seen_ids = []
        for marker_corners, marker_id in zip(corners, ids.flatten()):
            marker_id = int(marker_id)
            if marker_id not in self.marker_points:
                continue

            image_points = marker_corners.reshape((4, 2)).astype(np.float32)
            object_points = self.marker_points[marker_id]
            all_object_points.append(object_points)
            all_image_points.append(image_points)
            seen_ids.append(marker_id)

        if not seen_ids:
            return

        if self.localization_locked:
            return

        x = y = yaw = None
        object_points = np.concatenate(all_object_points, axis=0).astype(np.float32)
        image_points = np.concatenate(all_image_points, axis=0).astype(np.float32)

        if len(seen_ids) >= 2:
            try:
                success, rvec, tvec, inliers = cv2.solvePnPRansac(
                    object_points,
                    image_points,
                    self.camera_matrix,
                    self.dist_coeffs,
                    reprojectionError=self.ransac_reproj_error,
                    iterationsCount=100,
                    flags=cv2.SOLVEPNP_ITERATIVE,
                )
                if success:
                    if (
                        inliers is not None
                        and len(inliers) >= 4
                        and hasattr(cv2, 'solvePnPRefineLM')
                    ):
                        inlier_indices = inliers.reshape(-1)
                        rvec, tvec = cv2.solvePnPRefineLM(
                            object_points[inlier_indices],
                            image_points[inlier_indices],
                            self.camera_matrix,
                            self.dist_coeffs,
                            rvec,
                            tvec,
                        )
                    x, y, yaw = self._map_to_odom_from_pnp(
                        rvec,
                        tvec,
                        camera_T_base,
                        odom_T_base,
                    )
            except cv2.error:
                x = y = yaw = None

        if x is None:
            estimates = []
            for marker_corners, marker_id in zip(corners, ids.flatten()):
                marker_id = int(marker_id)
                if marker_id not in self.marker_points:
                    continue

                image_points = marker_corners.reshape((4, 2)).astype(np.float32)
                object_points = self.marker_points[marker_id]
                success, rvec, tvec = cv2.solvePnP(
                    object_points,
                    image_points,
                    self.camera_matrix,
                    self.dist_coeffs,
                    flags=cv2.SOLVEPNP_ITERATIVE,
                )
                if not success:
                    continue

                est_x, est_y, est_yaw = self._map_to_odom_from_pnp(
                    rvec,
                    tvec,
                    camera_T_base,
                    odom_T_base,
                )
                distance = float(np.linalg.norm(tvec.reshape(3)))
                weight = 1.0 / max(distance, 0.25)
                estimates.append((est_x, est_y, est_yaw, weight))

            if not estimates:
                return

            sum_w = sum(weight for _, _, _, weight in estimates)
            x = sum(px * w for px, _, _, w in estimates) / sum_w
            y = sum(py * w for _, py, _, w in estimates) / sum_w
            yaw_sin = sum(math.sin(pyaw) * w for _, _, pyaw, w in estimates) / sum_w
            yaw_cos = sum(math.cos(pyaw) * w for _, _, pyaw, w in estimates) / sum_w
            yaw = math.atan2(yaw_sin, yaw_cos)

        candidate_x = x + self.map_x_offset
        candidate_y = y + self.map_y_offset
        candidate_yaw = math.atan2(
            math.sin(yaw + self.map_yaw_offset),
            math.cos(yaw + self.map_yaw_offset),
        )

        if self._recently_moving():
            self._skipped_for_motion_count += 1
            if self._skipped_for_motion_count == 1 or self._skipped_for_motion_count % 20 == 0:
                self.get_logger().info(
                    'aruco_map_localizer: keeping current map->odom while robot is moving '
                    f'(v={self.current_linear_speed:.2f} m/s, w={self.current_angular_speed:.2f} rad/s)'
                )
            return

        self._skipped_for_motion_count = 0

        if not self._candidate_jump_is_reasonable(candidate_x, candidate_y, candidate_yaw):
            self._rejected_jump_count += 1
            if self._rejected_jump_count == 1 or self._rejected_jump_count % 10 == 0:
                last_x, last_y, last_yaw = self.last_map_to_odom
                translation_jump = math.hypot(candidate_x - last_x, candidate_y - last_y)
                yaw_jump = abs(angle_diff(candidate_yaw, last_yaw))
                self.get_logger().warn(
                    'aruco_map_localizer: rejected pose jump from markers '
                    f'{sorted(set(seen_ids))} (dx={translation_jump:.2f} m, dyaw={yaw_jump:.2f} rad)'
                )
            return

        self._rejected_jump_count = 0

        x, y, yaw = candidate_x, candidate_y, candidate_yaw
        if self.last_map_to_odom is not None:
            last_x, last_y, last_yaw = self.last_map_to_odom
            alpha = min(max(self.smoothing_alpha, 0.0), 1.0)
            x = alpha * x + (1.0 - alpha) * last_x
            y = alpha * y + (1.0 - alpha) * last_y
            yaw = math.atan2(
                alpha * math.sin(yaw) + (1.0 - alpha) * math.sin(last_yaw),
                alpha * math.cos(yaw) + (1.0 - alpha) * math.cos(last_yaw),
            )

        self.last_map_to_odom = (x, y, yaw)
        stamp = self._now_msg()
        self.last_publish_stamp = stamp
        self._publish_tf(stamp, x, y, yaw)
        self._publish_pose(stamp, x, y, yaw)

        self._last_detection_count += 1
        if self._last_detection_count == 1 or self._last_detection_count % 20 == 0:
            self.get_logger().info(
                f'aruco_map_localizer: localized from markers {sorted(set(seen_ids))} '
                f'-> map->odom ({x:.2f}, {y:.2f}, {yaw:.2f} rad)'
            )
            if self.lock_after_localization and not self.goal_active:
                self.get_logger().info(
                    'aruco_map_localizer: localization is currently unlocked and may refine until a navigation goal starts'
                )

    def _publish_tf(self, stamp, x, y, yaw):
        msg = TransformStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = self.map_frame
        msg.child_frame_id = self.odom_frame
        msg.transform.translation.x = float(x)
        msg.transform.translation.y = float(y)
        msg.transform.translation.z = 0.0
        qx, qy, qz, qw = quaternion_from_yaw(yaw)
        msg.transform.rotation.x = qx
        msg.transform.rotation.y = qy
        msg.transform.rotation.z = qz
        msg.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(msg)

    def _publish_pose(self, stamp, x, y, yaw):
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = self.map_frame
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.position.z = 0.0
        qx, qy, qz, qw = quaternion_from_yaw(yaw)
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        msg.pose.covariance[0] = 0.05
        msg.pose.covariance[7] = 0.05
        msg.pose.covariance[35] = 0.10
        self.pose_pub.publish(msg)

    def _republish_last_tf(self):
        if self.last_map_to_odom is None:
            return
        now = self._now_msg()
        self.last_publish_stamp = now
        self._publish_tf(now, *self.last_map_to_odom)


def main():
    rclpy.init()
    node = ArucoMapLocalizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
