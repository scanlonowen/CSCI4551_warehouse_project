"""
Launch file: Teleop for the forklift robot.
Publishes Twist on /cmd_vel; the twist_to_stamped relay (started by
warehouse_simulation.launch.py) converts it to TwistStamped on
/diff_cont/cmd_vel, which is what Jazzy's diff_drive_controller expects.
Usage: ros2 launch warehouse_robot_bringup teleop.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            name='teleop',
            output='screen',
            prefix='xterm -e',
            parameters=[{'use_sim_time': True}],
        ),
    ])
