"""
Launch slam_toolbox in online-async mode with the forklift params.
Run alongside warehouse_simulation.launch.py and teleop.launch.py:
    ros2 launch warehouse_robot_bringup warehouse_simulation.launch.py
    ros2 launch warehouse_robot_bringup slam.launch.py
    ros2 launch warehouse_robot_bringup teleop.launch.py
Drive the robot around, then save the map with:
    ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \
      "{name: {data: 'src/warehouse_robot_bringup/maps/warehouse'}}"
    ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
      "{filename: 'src/warehouse_robot_bringup/maps/warehouse'}"
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bringup_pkg = get_package_share_directory('warehouse_robot_bringup')
    slam_pkg = get_package_share_directory('slam_toolbox')

    default_slam_params = os.path.join(bringup_pkg, 'config', 'slam_toolbox_async.yaml')
    slam_params = LaunchConfiguration('slam_params_file', default=default_slam_params)
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_pkg, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam_params_file': slam_params,
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock',
        ),
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=default_slam_params,
            description='Full path to the slam_toolbox parameters file',
        ),
        slam,
    ])
