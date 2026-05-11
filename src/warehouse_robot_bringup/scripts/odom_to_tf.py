#!/usr/bin/env python3
"""
Publish odom -> base TF from a nav_msgs/Odometry topic.

The warehouse simulation uses this as the single TF source for odometry so
launch files can swap between wheel odom and Gazebo ground-truth odom without
creating competing TF parents for the robot base.
"""

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped


class OdomToTf(Node):
    def __init__(self):
        super().__init__('odom_to_tf')
        self.declare_parameter('odom_topic', '/diff_cont/odom')
        self.declare_parameter('odom_frame', '')
        self.declare_parameter('base_frame', '')

        self.odom_topic = self.get_parameter('odom_topic').value
        self.odom_frame_override = self.get_parameter('odom_frame').value
        self.base_frame_override = self.get_parameter('base_frame').value

        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(Odometry, self.odom_topic, self._odom_cb, 10)
        self.get_logger().info(
            f'Publishing TF from odometry topic {self.odom_topic}'
        )

    def _odom_cb(self, msg: Odometry) -> None:
        tf = TransformStamped()
        tf.header.stamp = msg.header.stamp
        tf.header.frame_id = self.odom_frame_override or msg.header.frame_id
        tf.child_frame_id = self.base_frame_override or msg.child_frame_id
        tf.transform.translation.x = msg.pose.pose.position.x
        tf.transform.translation.y = msg.pose.pose.position.y
        tf.transform.translation.z = msg.pose.pose.position.z
        tf.transform.rotation = msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(tf)


def main():
    rclpy.init()
    node = OdomToTf()
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
