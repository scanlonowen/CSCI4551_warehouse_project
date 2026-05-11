#!/usr/bin/env python3
"""
Filter the raw forklift LiDAR into three ROS scan topics:

  /scan
      Tight forward arc for Nav2 and collision_monitor so the forklift does
      not treat its own mast / body as an obstacle.

  /scan_slam
      Slightly wider than /scan so slam_toolbox keeps more side coverage
      during in-place turns without reintroducing the forklift's own body.

  /scan_localization
      Much wider than /scan so AMCL can use side / near-rear structure in
      repetitive warehouse aisles while still masking the forklift's direct
      rear self-hits.
"""
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

NAV_MASK_DEG = 96.0
SLAM_MASK_DEG = 100.0
LOCALIZATION_MASK_DEG = 150.0
NAV_MASK_RAD = math.radians(NAV_MASK_DEG)
SLAM_MASK_RAD = math.radians(SLAM_MASK_DEG)
LOCALIZATION_MASK_RAD = math.radians(LOCALIZATION_MASK_DEG)


class ScanFilter(Node):
    def __init__(self):
        super().__init__('scan_filter')
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

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

        self.nav_pub = self.create_publisher(LaserScan, '/scan', pub_qos)
        self.slam_pub = self.create_publisher(LaserScan, '/scan_slam', pub_qos)
        self.localization_pub = self.create_publisher(
            LaserScan, '/scan_localization', pub_qos,
        )
        self.create_subscription(LaserScan, '/scan_raw', self.cb, sub_qos)
        self._count = 0
        self.get_logger().info(
            'scan_filter ready: /scan_raw -> /scan '
            f'(|angle| > {NAV_MASK_DEG} deg masked) and /scan_slam '
            f'(|angle| > {SLAM_MASK_DEG} deg masked) and '
            f'/scan_localization (|angle| > {LOCALIZATION_MASK_DEG} deg masked)'
        )

    @staticmethod
    def _masked_scan(msg: LaserScan, mask_rad: float) -> LaserScan:
        out = LaserScan()
        out.header = msg.header
        out.angle_min = msg.angle_min
        out.angle_max = msg.angle_max
        out.angle_increment = msg.angle_increment
        out.time_increment = msg.time_increment
        out.scan_time = msg.scan_time
        out.range_min = msg.range_min
        out.range_max = msg.range_max

        no_hit = msg.range_max + 1.0
        ranges = list(msg.ranges)
        for i, _ in enumerate(ranges):
            angle = msg.angle_min + i * msg.angle_increment
            if abs(angle) > mask_rad:
                ranges[i] = no_hit

        out.ranges = ranges
        out.intensities = list(msg.intensities)
        return out

    def cb(self, msg: LaserScan):
        self.nav_pub.publish(self._masked_scan(msg, NAV_MASK_RAD))
        self.slam_pub.publish(self._masked_scan(msg, SLAM_MASK_RAD))
        self.localization_pub.publish(
            self._masked_scan(msg, LOCALIZATION_MASK_RAD)
        )
        self._count += 1
        if self._count == 1 or self._count % 50 == 0:
            self.get_logger().info(f'scan_filter: processed {self._count} scans')


def main():
    rclpy.init()
    node = ScanFilter()
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
