#!/usr/bin/env python3
"""
Relay node: converts geometry_msgs/Twist on /cmd_vel
to geometry_msgs/TwistStamped on /diff_cont/cmd_vel.

Needed because ROS2 Jazzy diff_drive_controller only accepts TwistStamped,
but teleop_twist_keyboard and Nav2 publish unstamped Twist.

The node also republishes the last received command at a fixed rate for a
short hold time. This keeps keyboard teleop smooth instead of relying on OS
key-repeat bursts to continuously drive the robot.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped


class TwistToStamped(Node):
    def __init__(self):
        super().__init__('twist_to_stamped')
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('cmd_hold_timeout', 0.30)
        self.declare_parameter('frame_id', 'drive_base_link')

        self.publish_rate = float(self.get_parameter('publish_rate').value)
        self.cmd_hold_timeout = float(self.get_parameter('cmd_hold_timeout').value)
        self.frame_id = self.get_parameter('frame_id').value

        self.sub = self.create_subscription(Twist, '/cmd_vel', self.callback, 10)
        self.pub = self.create_publisher(TwistStamped, '/diff_cont/cmd_vel', 10)
        self.latest_twist = Twist()
        self.last_cmd_time = None
        self.zero_sent = True
        timer_period = 1.0 / max(self.publish_rate, 1.0)
        self.timer = self.create_timer(timer_period, self.publish_latest)
        self.get_logger().info(
            'Relaying /cmd_vel (Twist) -> /diff_cont/cmd_vel (TwistStamped) '
            f'at {self.publish_rate:.1f} Hz with hold timeout '
            f'{self.cmd_hold_timeout:.2f} s'
        )

    def callback(self, msg: Twist):
        self.latest_twist = msg
        self.last_cmd_time = self.get_clock().now()
        self.zero_sent = False

    def publish_latest(self):
        if self.last_cmd_time is None:
            if not self.zero_sent:
                self.publish_twist(Twist())
                self.zero_sent = True
            return

        age = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        if age <= self.cmd_hold_timeout:
            self.publish_twist(self.latest_twist)
            self.zero_sent = False
        elif not self.zero_sent:
            self.publish_twist(Twist())
            self.zero_sent = True

    def publish_twist(self, msg: Twist):
        stamped = TwistStamped()
        stamped.header.stamp = self.get_clock().now().to_msg()
        stamped.header.frame_id = self.frame_id
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
