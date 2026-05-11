"""
Navigation mode using camera fiducials for global localization instead of AMCL.

This launches:
  1. Warehouse simulation with ground-truth odom -> drive_base_link TF
  2. Static map server
  3. ArUco camera localizer publishing map -> odom
  4. Nav2 navigation stack without localization nodes
  5. RViz
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_pkg = get_package_share_directory('warehouse_robot_bringup')
    nav2_pkg = get_package_share_directory('nav2_bringup')

    default_map = os.path.join(bringup_pkg, 'maps', 'warehouse.yaml')
    default_params = os.path.join(bringup_pkg, 'config', 'nav2_params.yaml')
    default_marker_config = os.path.join(
        bringup_pkg, 'config', 'fiducial_markers_big_warehouse.yaml'
    )

    map_yaml = LaunchConfiguration('map', default=default_map)
    params_file = LaunchConfiguration('params_file', default=default_params)
    marker_config = LaunchConfiguration('marker_config', default=default_marker_config)
    world = LaunchConfiguration('world', default='big_warehouse')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    autostart = LaunchConfiguration('autostart', default='true')
    raise_forks = LaunchConfiguration('raise_forks', default='true')
    startup_spin = LaunchConfiguration('startup_spin', default='true')
    startup_spin_delay = LaunchConfiguration('startup_spin_delay', default='12.0')
    startup_spin_duration = LaunchConfiguration('startup_spin_duration', default='14.0')
    startup_spin_speed = LaunchConfiguration('startup_spin_speed', default='0.45')
    map_x_offset = LaunchConfiguration('map_x_offset', default='-0.28')
    map_y_offset = LaunchConfiguration('map_y_offset', default='0.20')
    map_yaw_offset = LaunchConfiguration('map_yaw_offset', default='0.0')

    cleanup = ExecuteProcess(
        cmd=[os.path.join(bringup_pkg, 'scripts', 'cleanup_stale_procs.sh')],
        output='screen',
    )

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_pkg, 'launch', 'warehouse_simulation.launch.py')
        ),
        launch_arguments={
            'world': world,
            'use_ground_truth_odom': 'true',
        }.items(),
    )

    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_pkg, 'launch', 'view_robot.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            params_file,
            {'yaml_filename': map_yaml, 'use_sim_time': use_sim_time},
        ],
    )

    localization_lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': autostart},
            {'node_names': ['map_server']},
        ],
    )

    fiducial_localizer = Node(
        package='warehouse_robot_bringup',
        executable='aruco_map_localizer.py',
        name='aruco_map_localizer',
        output='screen',
        parameters=[{
            'marker_config': marker_config,
            'use_sim_time': use_sim_time,
            'map_frame': 'map',
            'odom_frame': 'odom',
            'base_frame': 'drive_base_link',
            'map_x_offset': map_x_offset,
            'map_y_offset': map_y_offset,
            'map_yaw_offset': map_yaw_offset,
        }],
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
            'use_localization': 'False',
        }.items(),
    )

    startup_spin_node = Node(
        package='warehouse_robot_bringup',
        executable='startup_spin.py',
        name='startup_spin',
        output='screen',
        condition=IfCondition(startup_spin),
        parameters=[{
            'use_sim_time': use_sim_time,
            'angular_speed': startup_spin_speed,
            'duration': startup_spin_duration,
            'publish_rate': 15.0,
            'cmd_topic': '/cmd_vel',
        }],
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

    raise_forks_after_spin = RegisterEventHandler(
        OnProcessExit(
            target_action=startup_spin_node,
            on_exit=[TimerAction(period=1.0, actions=[lift_forks])],
        )
    )

    raise_forks_without_spin = TimerAction(
        period=6.0,
        actions=[lift_forks],
        condition=UnlessCondition(startup_spin),
    )

    after_cleanup = RegisterEventHandler(
        OnProcessExit(
            target_action=cleanup,
            on_exit=[
                sim,
                rviz,
                TimerAction(period=2.0, actions=[map_server, localization_lifecycle, fiducial_localizer]),
                TimerAction(period=4.0, actions=[nav2]),
                TimerAction(period=startup_spin_delay, actions=[startup_spin_node]),
            ],
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=default_map,
                              description='Full path to map yaml'),
        DeclareLaunchArgument('params_file', default_value=default_params,
                              description='Full path to Nav2 params yaml'),
        DeclareLaunchArgument('marker_config', default_value=default_marker_config,
                              description='Marker layout yaml'),
        DeclareLaunchArgument('world', default_value='big_warehouse',
                              description='Warehouse world to load'),
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Use Gazebo simulation clock'),
        DeclareLaunchArgument('autostart', default_value='true',
                              description='Autostart Nav2 lifecycle nodes'),
        DeclareLaunchArgument(
            'startup_spin',
            default_value='true',
            description='Rotate once at startup so the camera can acquire a nearby door marker',
        ),
        DeclareLaunchArgument(
            'startup_spin_delay',
            default_value='12.0',
            description='Wall-clock delay before the one-shot startup spin begins (seconds)',
        ),
        DeclareLaunchArgument(
            'startup_spin_duration',
            default_value='14.0',
            description='How long to rotate during the one-shot startup spin (seconds)',
        ),
        DeclareLaunchArgument(
            'startup_spin_speed',
            default_value='0.45',
            description='Angular speed for the one-shot startup spin (rad/s)',
        ),
        DeclareLaunchArgument(
            'map_x_offset',
            default_value='-0.28',
            description='Fiducial localization X offset in map frame (meters)',
        ),
        DeclareLaunchArgument(
            'map_y_offset',
            default_value='0.20',
            description='Fiducial localization Y offset in map frame (meters)',
        ),
        DeclareLaunchArgument(
            'map_yaw_offset',
            default_value='0.0',
            description='Fiducial localization yaw offset in map frame (radians)',
        ),
        DeclareLaunchArgument(
            'raise_forks',
            default_value='true',
            description='Raise forks above the lidar scan plane during localization/navigation',
        ),
        cleanup,
        after_cleanup,
        raise_forks_after_spin,
        raise_forks_without_spin,
    ])
