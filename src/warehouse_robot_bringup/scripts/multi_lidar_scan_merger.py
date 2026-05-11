#!/usr/bin/env python3
"""
Merge the four directional forklift lidars into a single synthetic 360 scan.

Each input scan is transformed into a common output frame, then re-binned into
one full-circle LaserScan. This gives RViz / Nav2 / AMCL a single topic while
still letting the robot use multiple outward-facing sensors.

Robot self-returns are removed in the output frame using a rectangular body
footprint check. That keeps the full-ring scan usable for remapping without
the forklift localizing against its own chassis / mast.
"""

import math

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener


def quaternion_to_yaw(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def stamp_to_ns(stamp) -> int:
    return stamp.sec * 1_000_000_000 + stamp.nanosec


class MultiLidarScanMerger(Node):
    def __init__(self):
        super().__init__('multi_lidar_scan_merger')

        self.output_frame = self.declare_parameter(
            'output_frame', 'drive_base_link'
        ).value
        self.output_topic = self.declare_parameter(
            'output_topic', '/scan_raw'
        ).value
        self.publish_rate_hz = float(self.declare_parameter(
            'publish_rate_hz', 10.0
        ).value)
        self.sync_tolerance_sec = float(self.declare_parameter(
            'sync_tolerance_sec', 0.25
        ).value)
        self.output_angle_increment_deg = float(self.declare_parameter(
            'output_angle_increment_deg', 1.0
        ).value)
        self.self_filter_min_x = float(self.declare_parameter(
            'self_filter_min_x', -0.95
        ).value)
        self.self_filter_max_x = float(self.declare_parameter(
            'self_filter_max_x', 2.15
        ).value)
        self.self_filter_min_y = float(self.declare_parameter(
            'self_filter_min_y', -0.62
        ).value)
        self.self_filter_max_y = float(self.declare_parameter(
            'self_filter_max_y', 0.62
        ).value)
        self.self_filter_margin = float(self.declare_parameter(
            'self_filter_margin', 0.04
        ).value)
        sub_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        pub_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.scan_topics = {
            'front': '/scan_front_raw',
            'left': '/scan_left_raw',
            'right': '/scan_right_raw',
            'rear': '/scan_rear_raw',
        }
        self.latest_scans = {}
        self._transform_cache = {}
        self._warn_count = 0
        self._publish_count = 0

        for name, topic in self.scan_topics.items():
            self.create_subscription(
                LaserScan,
                topic,
                lambda msg, key=name: self._scan_cb(key, msg),
                sub_qos,
            )

        self.pub = self.create_publisher(LaserScan, self.output_topic, pub_qos)
        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)
        self.timer = self.create_timer(
            1.0 / max(self.publish_rate_hz, 1.0),
            self._publish_merged_scan,
        )

        self.get_logger().info(
            'multi_lidar_scan_merger ready: '
            '/scan_front_raw + /scan_left_raw + /scan_right_raw + /scan_rear_raw '
            f'-> {self.output_topic} in frame {self.output_frame}'
        )

    def _scan_cb(self, name: str, msg: LaserScan):
        self.latest_scans[name] = msg

    def _lookup_planar_transform(self, source_frame: str):
        cached = self._transform_cache.get(source_frame)
        if cached is not None:
            return cached

        transform = self.tf_buffer.lookup_transform(
            self.output_frame,
            source_frame,
            rclpy.time.Time(),
        )
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        yaw = quaternion_to_yaw(rotation)
        planar = (
            translation.x,
            translation.y,
            math.cos(yaw),
            math.sin(yaw),
        )
        self._transform_cache[source_frame] = planar
        return planar

    def _publish_merged_scan(self):
        if len(self.latest_scans) < len(self.scan_topics):
            return

        scans = [self.latest_scans[name] for name in self.scan_topics]
        newest_stamp_ns = max(stamp_to_ns(scan.header.stamp) for scan in scans)
        oldest_stamp_ns = min(stamp_to_ns(scan.header.stamp) for scan in scans)
        if newest_stamp_ns - oldest_stamp_ns > int(self.sync_tolerance_sec * 1e9):
            self._warn_count += 1
            if self._warn_count == 1 or self._warn_count % 20 == 0:
                self.get_logger().warn(
                    'multi_lidar_scan_merger: scans are too far apart in time; '
                    'waiting for a fresher synchronized set'
                )
            return

        output_scan = LaserScan()
        output_scan.header.stamp = max(scans, key=lambda scan: stamp_to_ns(
            scan.header.stamp
        )).header.stamp
        output_scan.header.frame_id = self.output_frame
        output_scan.angle_min = -math.pi
        output_scan.angle_increment = math.radians(
            self.output_angle_increment_deg
        )
        bin_count = int(
            round((2.0 * math.pi) / output_scan.angle_increment)
        ) + 1
        output_scan.angle_max = (
            output_scan.angle_min + (bin_count - 1) * output_scan.angle_increment
        )
        output_scan.time_increment = 0.0
        output_scan.scan_time = 1.0 / max(self.publish_rate_hz, 1.0)
        output_scan.range_min = min(scan.range_min for scan in scans)
        output_scan.range_max = max(scan.range_max for scan in scans)

        no_hit = output_scan.range_max + 1.0
        merged_ranges = [no_hit] * bin_count
        merged_intensities = [0.0] * bin_count

        try:
            for scan in scans:
                tx, ty, cos_yaw, sin_yaw = self._lookup_planar_transform(
                    scan.header.frame_id
                )
                for index, lidar_range in enumerate(scan.ranges):
                    if not math.isfinite(lidar_range):
                        continue
                    if lidar_range < scan.range_min or lidar_range > scan.range_max:
                        continue

                    local_angle = scan.angle_min + index * scan.angle_increment
                    local_x = lidar_range * math.cos(local_angle)
                    local_y = lidar_range * math.sin(local_angle)

                    world_x = tx + (cos_yaw * local_x - sin_yaw * local_y)
                    world_y = ty + (sin_yaw * local_x + cos_yaw * local_y)

                    if (
                        self.self_filter_min_x - self.self_filter_margin
                        <= world_x
                        <= self.self_filter_max_x + self.self_filter_margin
                        and self.self_filter_min_y - self.self_filter_margin
                        <= world_y
                        <= self.self_filter_max_y + self.self_filter_margin
                    ):
                        continue

                    merged_range = math.hypot(world_x, world_y)
                    if (
                        merged_range < output_scan.range_min
                        or merged_range > output_scan.range_max
                    ):
                        continue

                    merged_angle = math.atan2(world_y, world_x)
                    merged_index = int(round(
                        (merged_angle - output_scan.angle_min)
                        / output_scan.angle_increment
                    ))
                    if merged_index < 0 or merged_index >= bin_count:
                        continue

                    if merged_range < merged_ranges[merged_index]:
                        merged_ranges[merged_index] = merged_range
        except TransformException as exc:
            self._warn_count += 1
            if self._warn_count == 1 or self._warn_count % 20 == 0:
                self.get_logger().warn(
                    f'multi_lidar_scan_merger: waiting for TF: {exc}'
                )
            return

        output_scan.ranges = merged_ranges
        output_scan.intensities = merged_intensities
        self.pub.publish(output_scan)

        self._publish_count += 1
        if self._publish_count == 1 or self._publish_count % 50 == 0:
            self.get_logger().info(
                f'multi_lidar_scan_merger: published {self._publish_count} merged scans'
            )


def main():
    rclpy.init()
    node = MultiLidarScanMerger()
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
