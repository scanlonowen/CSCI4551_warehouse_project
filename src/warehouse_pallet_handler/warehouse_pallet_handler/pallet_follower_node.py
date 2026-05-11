#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32, Float64MultiArray
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger

class PalletFollower(Node):
    def __init__(self):
        super().__init__('pallet_follower')

        self.offset_sub = self.create_subscription(
            Float32,
            '/pallet_offset',
            self.offset_cb,
            10
        )

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.fork_pub = self.create_publisher(
            Float64MultiArray,
            '/fork_position_controller/commands',
            10
        )

        self.last_offset = None
        self.timer = self.create_timer(0.1, self.control_loop)

        self.center_tolerance = 0.05
        self.forward_speed = 0.16
        self.turn_speed = 0.25

        self.get_logger().info('pallet_follower started')

        self.set_forks(0.0)

        self.y_sub = self.create_subscription(Float32, '/pallet_y', self.y_cb, 10)
        self.attach_client = self.create_client(Trigger, '/pallet/attach')
        self.last_y = 0.0
        self.attached = False
        self.pickup_y_threshold = 474.0
        self.close_frames_required = 8
        self.close_frames = 0

    def offset_cb(self, msg):
        self.last_offset = msg.data

    def set_forks(self, height):
        msg = Float64MultiArray()
        msg.data = [height]
        self.fork_pub.publish(msg)

    def height_cb(self, msg):
        self.last_height = msg.data

    def y_cb(self, msg):
        self.last_y = msg.data

    def control_loop(self):
        cmd = Twist()

        if self.attached:
            return

        if self.last_y > self.pickup_y_threshold and abs(self.last_offset or 0.0) < 0.12:
            self.close_frames += 1
        else:
            self.close_frames = 0

        if self.close_frames >= self.close_frames_required:
            self.get_logger().info('Pallet is near, attempting attach and lift')

            self.cmd_pub.publish(Twist())

            if self.attach_client.wait_for_service(timeout_sec=1.0):
                req = Trigger.Request()
                self.attach_client.call_async(req)
            else:
                self.get_logger().warn('/pallet/attach service is not available')

            self.set_forks(0.2)
            self.attached = True
            return

        if self.last_offset is None:
            cmd.angular.z = 0.2
            self.cmd_pub.publish(cmd)
            return

        offset = self.last_offset

        if abs(offset) > self.center_tolerance:
            cmd.angular.z = -self.turn_speed * offset
            cmd.linear.x = 0.0
        else:
            cmd.angular.z = 0.0
            cmd.linear.x = self.forward_speed

        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = PalletFollower()

    try: 
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop = Twist()
        node.cmd_pub.publish(stop)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

