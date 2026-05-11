"""
Pallet detector node.

Subscribes:
  /camera/image_raw  (sensor_msgs/Image)        — forklift RGB camera
  /camera/camera_info (sensor_msgs/CameraInfo)  — for intrinsics (fx, cx)
  /scan              (sensor_msgs/LaserScan)    — for range estimation

Publishes:
  /pallet_pose       (geometry_msgs/PoseStamped, frame_id=map)

Pipeline (intentionally lightweight — no PnP, no neural net):
  1. HSV-threshold the image to find pallet-colored blobs.
  2. Filter contours by area and aspect ratio (pallets are wider than tall in image).
  3. For the strongest candidate, project its centroid pixel-column to a yaw bearing
     using the camera intrinsics (fx, cx), then sample the LaserScan range at that
     bearing. This is a 2D, lidar-assisted fix — NOT a true 3D pose estimate.
     It assumes the lidar and camera are roughly co-axial in yaw, which is true
     for this forklift (both forward-facing, mounted near the chassis centerline).
  4. Build a PoseStamped in camera_link_optical, transform to map via tf2,
     and publish — throttled to keep downstream consumers happy.

Known limitations / TODOs:
  * HSV thresholds were chosen by inspecting the texture PNG of the
    aws_robomaker_warehouse_PalletJackB_01 model (orange-yellow deck).
    They WILL need to be re-tuned in the running sim, especially under
    different lighting.
  * Single-pallet assumption: only the largest valid blob is published.
  * No multi-frame association / tracking — "stability" is just an
    N-of-M consecutive-detection counter.
  * Bearing-from-pixel assumes a pinhole camera with no radial distortion,
    which Gazebo's default camera honors.
  * Range is taken from a single LaserScan beam; no clustering. If the
    pallet is partly occluded the distance can be wrong.
"""

import math
from typing import Optional

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from cv_bridge import CvBridge
from std_msgs.msg import Float32

from sensor_msgs.msg import Image, CameraInfo, LaserScan
from geometry_msgs.msg import PoseStamped, Quaternion

import tf2_ros
from tf2_ros import TransformException
# Importing tf2_geometry_msgs registers PoseStamped with the tf2 buffer's
# do_transform machinery. The import alone is what matters.
import tf2_geometry_msgs  # noqa: F401


# ---------------------------------------------------------------------------
# HSV thresholds for the warehouse_pallet model (brown wood, RGB ~ (140, 95, 55)
# diffuse / (110, 75, 45) ambient — see models/warehouse_pallet/model.sdf).
# In OpenCV HSV (H in 0..179):
#   H ~ 8-25    (brown sits on the warm orange-amber band)
#   S ~ 60-220  (mid-saturation; brown is less saturated than orange)
#   V ~ 40-200  (allow shadow / specular variation)
# Re-tune against live sim frames if detection misses or false-positives.
HSV_LOWER = np.array([10, 20, 100], dtype=np.uint8)
HSV_UPPER = np.array([30, 180, 255], dtype=np.uint8)

# Contour filters
MIN_CONTOUR_AREA_PX = 50        # reject specks
MAX_CONTOUR_AREA_PX = 500_000    # reject "the whole image is yellow"
MIN_ASPECT_RATIO = 0.1         # pallet is wider than tall
MAX_ASPECT_RATIO = 50.0

# Detection stability
STABLE_FRAMES_REQUIRED = 3       # require N consecutive detections before publishing
PUBLISH_RATE_HZ = 2.0


def yaw_to_quaternion(yaw: float) -> Quaternion:
    """Build a Quaternion from a planar yaw."""
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


class PalletDetector(Node):

    def __init__(self) -> None:
        super().__init__('pallet_detector')

        # Frame names — these match the URDF in warehouse_robot_bringup.
        self.camera_optical_frame = self.declare_parameter(
            'camera_optical_frame', 'camera_link_optical'
        ).get_parameter_value().string_value
        self.target_frame = self.declare_parameter(
            'target_frame', 'map'
        ).get_parameter_value().string_value

        self.bridge = CvBridge()

        # Camera intrinsics — populated on first CameraInfo message.
        self.fx: Optional[float] = None
        self.cx: Optional[float] = None
        self.image_width: Optional[int] = None

        # Last LaserScan (kept around so image callbacks can sample it).
        self.last_scan: Optional[LaserScan] = None

        # Stability counter
        self._consecutive_detections = 0

        # Throttle
        self._min_publish_period = 1.0 / PUBLISH_RATE_HZ
        self._last_publish_time = 0.0

        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.create_subscription(
            CameraInfo, '/camera/camera_info', self._camera_info_cb, sensor_qos
        )
        self.create_subscription(
            LaserScan, '/scan', self._scan_cb, sensor_qos
        )
        self.create_subscription(
            Image, '/camera/image_raw', self._image_cb, sensor_qos
        )

        self.pose_pub = self.create_publisher(PoseStamped, '/pallet_pose', 10)

        self.get_logger().info('pallet_detector started')

        self.offset_pub = self.create_publisher(Float32, '/pallet_offset', 10)

        self.mask_pub = self.create_publisher(Image, '/pallet_mask', 10)

        self.height_pub = self.create_publisher(Float32, '/pallet_height', 10)

        self.y_pub = self.create_publisher(Float32, '/pallet_y', 10)
    # ---------------- callbacks ----------------

    def _camera_info_cb(self, msg: CameraInfo) -> None:
        # K = [fx 0 cx; 0 fy cy; 0 0 1]
        if self.fx is None and len(msg.k) >= 9 and msg.k[0] > 0.0:
            self.fx = float(msg.k[0])
            self.cx = float(msg.k[2])
            self.image_width = int(msg.width)
            self.get_logger().info(
                f'Got camera intrinsics: fx={self.fx:.1f} cx={self.cx:.1f} w={self.image_width}'
            )

    def _scan_cb(self, msg: LaserScan) -> None:
        self.last_scan = msg

    def _image_cb(self, msg: Image) -> None:
        if self.fx is None or self.cx is None:
            return  # need intrinsics first
        # if self.last_scan is None:
        #     return  # need lidar for range

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:  # cv_bridge.CvBridgeError, etc.
            self.get_logger().warn(f'cv_bridge failed: {e}')
            return

        candidate = self._find_best_pallet_blob(frame)
        if candidate is None:
            self.get_logger().info('No pallet blob found', throttle_duration_sec=1.0)
            self._consecutive_detections = 0
            return
        else:
            self.get_logger().info(f'Pallet candidate: {candidate}', throttle_duration_sec=1.0)

        cx_px, _cy_px, _w, _h = candidate
        offset = Float32()
        offset.data = float((cx_px - self.cx) / self.cx)
        self.offset_pub.publish(offset)
        height_msg = Float32()
        height_msg.data = float(_h)
        self.height_pub.publish(height_msg)
        y_msg = Float32()
        y_msg.data = float(_cy_px)
        self.y_pub.publish(y_msg)

        self._consecutive_detections += 1
        if self._consecutive_detections < STABLE_FRAMES_REQUIRED:
            return

        # Throttle
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        if (now_sec - self._last_publish_time) < self._min_publish_period:
            return

        # Pixel column -> bearing in camera frame.
        # For a pinhole camera: tan(bearing) = (u - cx) / fx.
        # Positive bearing = pixel right of center = +X in optical frame.
        bearing = math.atan2(cx_px - self.cx, self.fx)

        # Sample the laser scan at the same bearing.
        # NOTE: assumes lidar and camera are co-axial in yaw. The lidar's
        # 0 rad is straight forward (matches camera optical Z-forward).
        rng = self._range_at_bearing(self.last_scan, bearing)
        if rng is None or not math.isfinite(rng):
            return

        # Build PoseStamped in camera_link_optical.
        # Optical frame convention: Z forward, X right, Y down.
        # Place a point at distance `rng` along the bearing in the X-Z plane.
        pose_cam = PoseStamped()
        pose_cam.header.stamp = msg.header.stamp
        pose_cam.header.frame_id = self.camera_optical_frame
        pose_cam.pose.position.x = rng * math.sin(bearing)
        pose_cam.pose.position.y = 0.0
        pose_cam.pose.position.z = rng * math.cos(bearing)
        # Orientation in optical frame is meaningless here (we don't know
        # the pallet's yaw from a blob). Set identity.
        pose_cam.pose.orientation.w = 1.0

        # Transform to map.
        try:
            pose_map = self.tf_buffer.transform(
                pose_cam,
                self.target_frame,
                timeout=rclpy.duration.Duration(seconds=0.2),
            )
        except TransformException as ex:
            self.get_logger().warn(
                f'TF {self.camera_optical_frame} -> {self.target_frame} failed: {ex}',
                throttle_duration_sec=2.0,
            )
            return

        # Replace orientation with identity yaw in map (we don't estimate it).
        # TODO: derive pallet yaw from contour minAreaRect once we trust the
        # pixel-to-world mapping.
        pose_map.pose.orientation = yaw_to_quaternion(0.0)

        self.pose_pub.publish(pose_map)
        self._last_publish_time = now_sec

    # ---------------- helpers ----------------

    def _find_best_pallet_blob(self, frame_bgr: np.ndarray):
        """Return (cx, cy, w, h) of the best pallet candidate, or None."""
        hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, HSV_LOWER, HSV_UPPER)
        mask_msg = self.bridge.cv2_to_imgmsg(mask, encoding='mono8')
        self.mask_pub.publish(mask_msg)

        # Light morphological cleanup.
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        best = None
        best_area = 0.0
        for c in contours:
            area = cv2.contourArea(c)
            if area < MIN_CONTOUR_AREA_PX or area > MAX_CONTOUR_AREA_PX:
                continue
            x, y, w, h = cv2.boundingRect(c)
            if h <= 0:
                continue
            ar = float(w) / float(h)
            if ar < MIN_ASPECT_RATIO or ar > MAX_ASPECT_RATIO:
                continue
            if area > best_area:
                best_area = area
                best = (x + w / 2.0, y + h / 2.0, w, h)
        return best

    def _range_at_bearing(self, scan: LaserScan, bearing: float) -> Optional[float]:
        """Pick the LaserScan range at the closest beam to `bearing` (rad)."""
        if scan is None:
            return None
        if scan.angle_increment <= 0.0 or len(scan.ranges) == 0:
            return None
        # Clamp the bearing to the scan's angular field of view.
        if bearing < scan.angle_min or bearing > scan.angle_max:
            return None
        idx = int(round((bearing - scan.angle_min) / scan.angle_increment))
        idx = max(0, min(len(scan.ranges) - 1, idx))
        # Average a small window around the chosen index for noise rejection.
        half = 2
        lo = max(0, idx - half)
        hi = min(len(scan.ranges), idx + half + 1)
        window = [
            r for r in scan.ranges[lo:hi]
            if math.isfinite(r) and scan.range_min <= r <= scan.range_max
        ]
        if not window:
            return None
        return float(np.median(window))


def main(args=None):
    rclpy.init(args=args)
    node = PalletDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
