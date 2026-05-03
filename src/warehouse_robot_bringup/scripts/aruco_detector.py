#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TwistStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
import math

class ArUcoNavigator(Node):
    def __init__(self):
        super().__init__('aruco_navigator')
        
        self.img_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.info_sub = self.create_subscription(CameraInfo, '/camera/camera_info', self.info_callback, 10)
        self.cmd_pub = self.create_publisher(TwistStamped, '/diff_cont/cmd_vel', 10)
        
        self.bridge = CvBridge()
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        self.parameters = cv2.aruco.DetectorParameters_create()
        
        # ==========================================
        # ROBOT GEOMETRY & TUNING (Adjust these!)
        # ==========================================
        # If it drives double the distance, your tag is likely 0.1m, not 0.2m!
        self.marker_size = 0.1  
        
        # Distance from the center of the wheels (base_link) forward to the camera lens
        self.cam_x_offset = 0.5 
        
        # Distance from the center of the wheels to the center of the pallet when fully parked
        # (e.g., half the robot's length + fork length + half the pallet's depth)
        self.docking_distance = 1.3  

        self.turn_speed = 0.3      # rad/s
        self.drive_speed = 0.4     # m/s
        # ==========================================
        
        self.cam_matrix = None
        self.dist_coeffs = None

        self.state = 'SEARCH'  
        self.phase_start_time = 0.0
        
        self.turn_90_duration = (math.pi / 2.0) / self.turn_speed 
        self.turn_dir = 1.0        
        self.lateral_duration = 0.0
        self.forward_duration = 0.0

        self.get_logger().info("Geometry Navigator Started. Waiting for Camera Info...")

    def info_callback(self, msg):
        if self.cam_matrix is None:
            self.cam_matrix = np.array(msg.k).reshape((3, 3))
            self.dist_coeffs = np.array(msg.d)
            self.get_logger().info("Camera Info Received!")

    def image_callback(self, msg):
        if self.cam_matrix is None:
            return

        try:
            drive_msg = TwistStamped()
            drive_msg.header.stamp = self.get_clock().now().to_msg()
            drive_msg.header.frame_id = 'base_link'

            current_time = self.get_clock().now().nanoseconds / 1e9

            if self.state == 'SEARCH':
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
                corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)
                
                if ids is not None and 0 in ids:
                    idx = np.where(ids == 0)[0][0]
                    _, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                        [corners[idx]], self.marker_size, self.cam_matrix, self.dist_coeffs
                    )
                    
                    cam_x = tvecs[0][0][0] 
                    cam_z = tvecs[0][0][2] 
                    
                    # MATH: Shift the coordinate system back to the wheels
                    base_x = cam_z + self.cam_x_offset
                    base_y = -cam_x  
                    
                    self.get_logger().info(f"Target is {base_x:.2f}m Forward, {base_y:.2f}m Lateral from WHEELS.")

                    self.turn_dir = 1.0 if base_y > 0 else -1.0
                    self.lateral_duration = abs(base_y) / self.drive_speed
                    
                    # MATH: Stop when the wheels are at the safe docking distance
                    travel_distance = max(0.0, base_x - self.docking_distance) 
                    self.forward_duration = travel_distance / self.drive_speed

                    self.get_logger().info("SNAPSHOT TAKEN! Executing Manhattan Path.")
                    self.state = 'TURN_90_OUT'
                    self.phase_start_time = current_time
                else:
                    self.get_logger().info("Searching for Marker 0...", throttle_duration_sec=1.0)
                    drive_msg.twist.linear.x = 0.0
                    drive_msg.twist.angular.z = 0.3

            elif self.state == 'TURN_90_OUT':
                if (current_time - self.phase_start_time) < self.turn_90_duration:
                    drive_msg.twist.angular.z = self.turn_speed * self.turn_dir
                else:
                    self.state = 'DRIVE_LATERAL'
                    self.phase_start_time = current_time

            elif self.state == 'DRIVE_LATERAL':
                if (current_time - self.phase_start_time) < self.lateral_duration:
                    drive_msg.twist.linear.x = self.drive_speed
                else:
                    self.state = 'TURN_90_IN'
                    self.phase_start_time = current_time

            elif self.state == 'TURN_90_IN':
                if (current_time - self.phase_start_time) < self.turn_90_duration:
                    drive_msg.twist.angular.z = self.turn_speed * (-self.turn_dir)
                else:
                    self.state = 'DRIVE_FORWARD'
                    self.phase_start_time = current_time

            elif self.state == 'DRIVE_FORWARD':
                if (current_time - self.phase_start_time) < self.forward_duration:
                    drive_msg.twist.linear.x = self.drive_speed
                else:
                    self.state = 'DONE'

            elif self.state == 'DONE':
                drive_msg.twist.linear.x = 0.0
                drive_msg.twist.angular.z = 0.0
                self.get_logger().info("PARKED! Ready to lift.", throttle_duration_sec=2.0)

            self.cmd_pub.publish(drive_msg)

        except Exception as e:
            self.get_logger().error(f"Error: {str(e)}")

def main():
    rclpy.init()
    node = ArUcoNavigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()