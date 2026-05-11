"""
Manual mapping launch for building a SLAM map in Gazebo.

This is intentionally smaller than explore.launch.py:
  1. Start Gazebo + forklift + sensor bridges.
  2. Start slam_toolbox online mapping.
  3. Start RViz.
  4. Start a slow teleop session for manual driving (optional).
  5. Raise the forks so the front lidar does not see the fork carriage.

The default teleop profile is intentionally slow for shelf-heavy mapping:
    ros2 launch warehouse_robot_bringup teleop.launch.py \
      speed:=0.15 turn:=0.25

Save the map when it looks good in RViz:
    ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \
      "{name: {data: 'src/warehouse_robot_bringup/maps/warehouse'}}"
    ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
      "{filename: 'src/warehouse_robot_bringup/maps/warehouse'}"
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
    slam_params = os.path.join(bringup_pkg, 'config', 'slam_toolbox_manual.yaml')

    world = LaunchConfiguration('world', default='big_warehouse')
    spawn_x = LaunchConfiguration('spawn_x', default='0.0')
    spawn_y = LaunchConfiguration('spawn_y', default='-16.0')
    spawn_z = LaunchConfiguration('spawn_z', default='0.1')
    spawn_yaw = LaunchConfiguration('spawn_yaw', default='1.5708')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    rviz = LaunchConfiguration('rviz', default='true')
    teleop = LaunchConfiguration('teleop', default='true')
    teleop_speed = LaunchConfiguration('teleop_speed', default='0.15')
    teleop_turn = LaunchConfiguration('teleop_turn', default='0.25')
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
            'world': world,
            'spawn_x': spawn_x,
            'spawn_y': spawn_y,
            'spawn_z': spawn_z,
            'spawn_yaw': spawn_yaw,
            'use_ground_truth_odom': 'true',
        }.items(),
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_pkg, 'launch', 'slam.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam_params_file': slam_params,
        }.items(),
    )

    rviz_inc = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_pkg, 'launch', 'view_robot.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
        condition=IfCondition(rviz),
    )

    teleop_inc = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_pkg, 'launch', 'teleop.launch.py')
        ),
        launch_arguments={
            'speed': teleop_speed,
            'turn': teleop_turn,
        }.items(),
        condition=IfCondition(teleop),
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
                slam,
                rviz_inc,
                teleop_inc,
                TimerAction(period=12.0, actions=[lift_forks]),
            ],
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value='big_warehouse',
            description='World to load: big_warehouse, small_warehouse, or no_roof_small_warehouse',
        ),
        DeclareLaunchArgument('spawn_x', default_value='0.0'),
        DeclareLaunchArgument('spawn_y', default_value='-16.0'),
        DeclareLaunchArgument('spawn_z', default_value='0.1'),
        DeclareLaunchArgument('spawn_yaw', default_value='1.5708'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument(
            'teleop',
            default_value='true',
            description='Launch keyboard teleop in a separate xterm window',
        ),
        DeclareLaunchArgument(
            'teleop_speed',
            default_value='0.15',
            description='Default linear speed for manual mapping teleop',
        ),
        DeclareLaunchArgument(
            'teleop_turn',
            default_value='0.25',
            description='Default angular speed for manual mapping teleop',
        ),
        DeclareLaunchArgument(
            'raise_forks',
            default_value='true',
            description='Raise forks above the lidar scan plane during mapping',
        ),
        cleanup,
        after_cleanup,
    ])
