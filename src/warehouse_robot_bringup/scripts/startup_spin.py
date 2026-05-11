#!/usr/bin/env python3
"""
Publish a short in-place rotation on /cmd_vel and then exit.

Used by the fiducial navigation launch so the forklift can automatically scan
for nearby ArUco markers after spawn without any manual teleop.
"""

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class StartupSpin(Node):
    def __init__(self):
        super().__init__('startup_spin')
        self.declare_parameter('angular_speed', 0.45)
        self.declare_parameter('linear_speed', 0.0)
        self.declare_parameter('duration', 14.0)
        self.declare_parameter('publish_rate', 15.0)
        self.declare_parameter('cmd_topic', '/cmd_vel')

        self.angular_speed = float(self.get_parameter('angular_speed').value)
        self.linear_speed = float(self.get_parameter('linear_speed').value)
        self.duration = float(self.get_parameter('duration').value)
        self.publish_rate = max(float(self.get_parameter('publish_rate').value), 1.0)
        self.cmd_topic = str(self.get_parameter('cmd_topic').value)

        self.publisher = self.create_publisher(Twist, self.cmd_topic, 10)
        self.start_time = None
        self.finish_countdown = None
        self.timer = self.create_timer(1.0 / self.publish_rate, self._on_timer)

        self.get_logger().info(
            f'startup_spin ready: {self.duration:.1f}s at '
            f'{self.angular_speed:.2f} rad/s on {self.cmd_topic}'
        )

    def _publish(self, linear_x: float, angular_z: float):
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self.publisher.publish(msg)

    def _on_timer(self):
        now = self.get_clock().now()
        if now.nanoseconds == 0:
            return

        if self.start_time is None:
            self.start_time = now
            self.finish_countdown = max(int(self.publish_rate * 0.5), 2)
            self.get_logger().info('startup_spin: beginning one-shot localization sweep')

        elapsed = (now - self.start_time).nanoseconds / 1e9
        if elapsed < self.duration:
            self._publish(self.linear_speed, self.angular_speed)
            return

        if self.finish_countdown > 0:
            self._publish(0.0, 0.0)
            self.finish_countdown -= 1
            if self.finish_countdown == 0:
                self.get_logger().info('startup_spin: complete')
                self.timer.cancel()
                self.destroy_node()
                rclpy.shutdown()


def main():
    rclpy.init()
    node = StartupSpin()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
