"""
Stage 3: Localize on the saved map + Nav2 goal navigation.

Launches:
  1. warehouse_simulation.launch.py  (Gazebo + forklift + controllers + bridges)
  2. nav2_bringup/bringup_launch.py  (AMCL + planner + controller + bt_navigator)
  3. RViz with the forklift config

After launch:
  - In RViz, click "2D Pose Estimate" near the robot's actual pose so AMCL
    converges.
  - Click "Nav2 Goal" to send goals.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, TimerAction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bringup_pkg = get_package_share_directory('warehouse_robot_bringup')
    nav2_pkg = get_package_share_directory('nav2_bringup')

    default_map = os.path.join(bringup_pkg, 'maps', 'warehouse.yaml')
    default_params = os.path.join(bringup_pkg, 'config', 'nav2_params.yaml')

    map_yaml = LaunchConfiguration('map', default=default_map)
    params_file = LaunchConfiguration('params_file', default=default_params)
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    autostart = LaunchConfiguration('autostart', default='true')
    raise_forks = LaunchConfiguration('raise_forks', default='true')

    cleanup = ExecuteProcess(
        cmd=[os.path.join(bringup_pkg, 'scripts', 'cleanup_stale_procs.sh')],
        output='screen',
    )

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_pkg, 'launch', 'warehouse_simulation.launch.py')
        ),
        launch_arguments={
            'use_ground_truth_odom': 'true',
        }.items(),
    )

    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_pkg, 'launch', 'view_robot.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_pkg, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': map_yaml,
            'params_file': params_file,
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'use_composition': 'False',
        }.items(),
    )

    lift_forks = ExecuteProcess(
        cmd=[
            'ros2', 'topic', 'pub', '--once',
            '/fork_position_controller/commands',
            'std_msgs/msg/Float64MultiArray',
            '{data: [1.1]}',
        ],
        output='screen',
        condition=IfCondition(raise_forks),
    )

    after_cleanup = RegisterEventHandler(
        OnProcessExit(
            target_action=cleanup,
            on_exit=[
                sim,
                rviz,
                TimerAction(period=3.0, actions=[nav2]),
                TimerAction(period=12.0, actions=[lift_forks]),
            ],
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=default_map,
                              description='Full path to map yaml'),
        DeclareLaunchArgument('params_file', default_value=default_params,
                              description='Full path to Nav2 params yaml'),
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Use Gazebo simulation clock'),
        DeclareLaunchArgument('autostart', default_value='true',
                              description='Autostart Nav2 lifecycle nodes'),
        DeclareLaunchArgument(
            'raise_forks',
            default_value='true',
            description='Raise forks above the lidar scan plane during localization/navigation',
        ),
        cleanup,
        after_cleanup,
    ])
