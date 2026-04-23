"""
Stage 4: Autonomous warehouse tour.

Includes the full Stage-3 navigation stack + the waypoint_follower node.
You still need to set the initial pose in RViz after launch so AMCL converges
before the tour starts.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    bringup_pkg = get_package_share_directory('warehouse_robot_bringup')

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_pkg, 'launch', 'navigation.launch.py')
        ),
    )

    # Delay the waypoint node so Nav2 + AMCL have a chance to come up before
    # waitUntilNav2Active() starts probing.
    waypoint_node = TimerAction(
        period=10.0,
        actions=[
            Node(
                package='warehouse_robot_bringup',
                executable='waypoint_follower.py',
                name='waypoint_follower',
                output='screen',
                parameters=[{'use_sim_time': True}],
            ),
        ],
    )

    return LaunchDescription([
        navigation,
        waypoint_node,
    ])
