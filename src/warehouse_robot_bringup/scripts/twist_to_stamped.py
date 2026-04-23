#!/usr/bin/env python3
"""
Relay node: converts geometry_msgs/Twist on /cmd_vel
to geometry_msgs/TwistStamped on /diff_cont/cmd_vel.

Needed because ROS2 Jazzy diff_drive_controller only accepts TwistStamped,
but teleop_twist_keyboard and Nav2 publish unstamped Twist.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped


class TwistToStamped(Node):
    def __init__(self):
        super().__init__('twist_to_stamped')
        self.sub = self.create_subscription(Twist, '/cmd_vel', self.callback, 10)
        self.pub = self.create_publisher(TwistStamped, '/diff_cont/cmd_vel', 10)
        self.get_logger().info('Relaying /cmd_vel (Twist) -> /diff_cont/cmd_vel (TwistStamped)')

    def callback(self, msg: Twist):
        stamped = TwistStamped()
        stamped.header.stamp = self.get_clock().now().to_msg()
        stamped.header.frame_id = 'base_link'
        stamped.twist = msg
        self.pub.publish(stamped)


def main():
    rclpy.init()
    node = TwistToStamped()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
