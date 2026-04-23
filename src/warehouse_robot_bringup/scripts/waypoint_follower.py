#!/usr/bin/env python3
"""
Stage 4: forklift waypoint tour.

Reads `config/waypoints.yaml` (named poses in the `map` frame) and drives the
forklift through the configured `route` using nav2_simple_commander.BasicNavigator.
Requires Nav2 + AMCL already running (e.g. from navigation.launch.py) and an
initial pose set in RViz so AMCL is converged.
"""

import math
import os

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.parameter import Parameter


def load_waypoints():
    pkg = get_package_share_directory('warehouse_robot_bringup')
    path = os.path.join(pkg, 'config', 'waypoints.yaml')
    with open(path) as f:
        data = yaml.safe_load(f)
    return data['route'], data['waypoints']


def make_pose(nav, x, y, yaw):
    p = PoseStamped()
    p.header.frame_id = 'map'
    p.header.stamp = nav.get_clock().now().to_msg()
    p.pose.position.x = float(x)
    p.pose.position.y = float(y)
    p.pose.orientation.z = math.sin(float(yaw) / 2.0)
    p.pose.orientation.w = math.cos(float(yaw) / 2.0)
    return p


def main():
    rclpy.init()
    nav = BasicNavigator()
    # BasicNavigator uses its own internal Node — push use_sim_time onto it so
    # its clock matches the rest of the Nav2 / Gazebo stack.
    nav.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])
    nav.get_logger().info('Waiting for Nav2 to become active...')
    nav.waitUntilNav2Active()
    nav.get_logger().info('Nav2 active — starting warehouse tour.')

    route, waypoints = load_waypoints()

    poses = []
    for name in route:
        if name not in waypoints:
            nav.get_logger().error(f'Unknown waypoint "{name}" in route; aborting.')
            rclpy.shutdown()
            return
        wp = waypoints[name]
        poses.append(make_pose(nav, wp['x'], wp['y'], wp.get('yaw', 0.0)))
        nav.get_logger().info(
            f"  {name}: x={wp['x']:.2f}  y={wp['y']:.2f}  yaw={wp.get('yaw', 0.0):.2f}"
        )

    nav.followWaypoints(poses)

    last_wp = -1
    while not nav.isTaskComplete():
        fb = nav.getFeedback()
        if fb is not None and fb.current_waypoint != last_wp:
            last_wp = fb.current_waypoint
            nav.get_logger().info(
                f'Heading to waypoint {last_wp + 1}/{len(poses)} '
                f'({route[last_wp]})'
            )

    result = nav.getResult()
    if result == TaskResult.SUCCEEDED:
        nav.get_logger().info('Tour complete — all waypoints reached.')
    elif result == TaskResult.CANCELED:
        nav.get_logger().warn('Tour canceled.')
    else:
        nav.get_logger().error(f'Tour failed (result={result}).')

    rclpy.shutdown()


if __name__ == '__main__':
    main()
