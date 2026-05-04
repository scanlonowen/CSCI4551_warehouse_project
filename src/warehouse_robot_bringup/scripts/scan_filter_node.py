#!/usr/bin/env python3
"""
Subscribes to /scan_raw and republishes on /scan with the rear angular arc
set to range_max + 1 (treated as "no hit" by Nav2 and collision_monitor).

The forklift's lift mast and chassis are located behind the LiDAR mount
(at x=1.30 m, the structure extends from x=1.19 m rearward).  This creates
self-hits at angles ±130°–±180°.  Those rays are masked here so the Nav2
costmap and collision_monitor never see the forklift as an obstacle.

SLAM subscribes to /scan_raw (unmasked) to retain full 360° loop-closure.
"""
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

MASK_DEG = 130.0   # mask everything beyond ±130° (rear arc)
MASK_RAD = math.radians(MASK_DEG)


class ScanFilter(Node):
    def __init__(self):
        super().__init__('scan_filter')
        # Match the bridge's RELIABLE QoS explicitly
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        qos = QoSProfile(depth=10,
                         reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)
        self.pub = self.create_publisher(LaserScan, '/scan', 10)
        self.create_subscription(LaserScan, '/scan_raw', self.cb, qos)
        self._count = 0
        self.get_logger().info(
            f'scan_filter ready: masking |angle| > {MASK_DEG}° on /scan_raw -> /scan')

    def cb(self, msg: LaserScan):
        out = LaserScan()
        out.header         = msg.header
        out.angle_min      = msg.angle_min
        out.angle_max      = msg.angle_max
        out.angle_increment = msg.angle_increment
        out.time_increment = msg.time_increment
        out.scan_time      = msg.scan_time
        out.range_min      = msg.range_min
        out.range_max      = msg.range_max

        no_hit = msg.range_max + 1.0   # any value > range_max = "no return"
        ranges = list(msg.ranges)
        for i, r in enumerate(ranges):
            angle = msg.angle_min + i * msg.angle_increment
            if abs(angle) > MASK_RAD:
                ranges[i] = no_hit
        out.ranges = ranges
        out.intensities = list(msg.intensities)
        self.pub.publish(out)
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
