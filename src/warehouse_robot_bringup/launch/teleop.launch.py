"""
Launch file: Teleop for the forklift robot.
Publishes Twist on /cmd_vel; the twist_to_stamped relay (started by
warehouse_simulation.launch.py) converts it to TwistStamped on
/diff_cont/cmd_vel, which is what Jazzy's diff_drive_controller expects.
Usage: ros2 launch warehouse_robot_bringup teleop.launch.py speed:=0.15 turn:=0.25
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    speed = LaunchConfiguration('speed')
    turn = LaunchConfiguration('turn')

    return LaunchDescription([
        DeclareLaunchArgument(
            'speed',
            default_value='0.5',
            description='Default linear speed for teleop_twist_keyboard',
        ),
        DeclareLaunchArgument(
            'turn',
            default_value='1.0',
            description='Default angular speed for teleop_twist_keyboard',
        ),
        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            name='teleop',
            output='screen',
            prefix='xterm -e',
            parameters=[{
                'use_sim_time': True,
                'speed': speed,
                'turn': turn,
            }],
        ),
    ])
