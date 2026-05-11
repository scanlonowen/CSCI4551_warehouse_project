"""
Stage 5.7: One-shot autonomous mission launch.

Phase 1 — Exploration (identical to explore.launch.py):
  Gazebo + forklift + SLAM + Nav2 (exploration config) + explore_lite.
  When exploration completes, exploration_map_saver saves the map and exits.

Phase 2 — Pick-and-deliver (starts automatically when Phase 1 map is saved):
  pallet_detector  — RGB + LiDAR pallet detection → /pallet_pose, /pallet_offset
  pallet_handler   — teleport-tracker for simulated pallet attachment
  mission          — state machine: go-to-entrance → approach → attach →
                     choose-dropoff → deliver → detach → print summary

The stack shuts down cleanly when the mission node exits.

Usage:
    ros2 launch warehouse_robot_bringup mission.launch.py

Optional args:
    map_basename:=<absolute_path>   where to write the explored-map files
    rviz:=false                     skip RViz
    use_sim_time:=true              (default; change only for hardware)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
    Shutdown,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _default_map_basename() -> str:
    share = get_package_share_directory('warehouse_robot_bringup')
    ws_root = os.path.abspath(os.path.join(share, '..', '..', '..', '..'))
    return os.path.join(
        ws_root, 'src', 'warehouse_robot_bringup', 'maps', 'warehouse_explored',
    )


def generate_launch_description():
    bringup_pkg = get_package_share_directory('warehouse_robot_bringup')
    nav2_pkg = get_package_share_directory('nav2_bringup')

    nav2_params = os.path.join(bringup_pkg, 'config', 'nav2_params_exploration.yaml')
    explore_params = os.path.join(bringup_pkg, 'config', 'explore_lite.yaml')

    map_basename = LaunchConfiguration('map_basename', default=_default_map_basename())
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    rviz_flag = LaunchConfiguration('rviz', default='true')

    # ── Pre-flight: kill stale processes from a previous run ─────────────────
    cleanup = ExecuteProcess(
        cmd=[os.path.join(bringup_pkg, 'scripts', 'cleanup_stale_procs.sh')],
        output='screen',
    )

    # ── Phase 1: Exploration (mirrors explore.launch.py) ─────────────────────

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_pkg, 'launch', 'warehouse_simulation.launch.py')
        ),
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_pkg, 'launch', 'slam.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    rviz_inc = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_pkg, 'launch', 'view_robot.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
        condition=IfCondition(rviz_flag),
    )

    # Nav2 in exploration mode (no AMCL — SLAM owns localization throughout).
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_pkg, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_params,
            'autostart': 'true',
            'use_composition': 'False',
        }.items(),
    )
    nav2_delayed = TimerAction(period=8.0, actions=[nav2])

    # Raise forks at t=12 s. The fork backing plate (fork_base_link) exits
    # the LiDAR scan plane (z=0.95 m above ground) only when the prismatic
    # joint exceeds 0.946 m. 1.1 m gives a 0.10 m clearance margin.
    lift_forks = ExecuteProcess(
        cmd=[
            'ros2', 'topic', 'pub', '--once',
            '/fork_position_controller/commands',
            'std_msgs/msg/Float64MultiArray',
            '{data: [1.1]}',
        ],
        output='screen',
    )
    lift_forks_delayed = TimerAction(period=12.0, actions=[lift_forks])

    # Seed spin at t=15 s: one full rotation builds an initial SLAM map so
    # explore_lite's first frontier search starts with a denser local map.
    seed_spin = ExecuteProcess(
        cmd=[
            'ros2', 'topic', 'pub', '--rate', '10', '--times', '130',
            '/cmd_vel', 'geometry_msgs/msg/Twist',
            '{linear: {x: 0.0}, angular: {z: 0.5}}',
        ],
        output='screen',
    )
    seed_spin_delayed = TimerAction(period=15.0, actions=[seed_spin])

    explore_node = Node(
        package='explore_lite',
        executable='explore',
        name='explore_node',
        output='screen',
        parameters=[explore_params, {'use_sim_time': use_sim_time}],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
        respawn=True,
        respawn_delay=5.0,
    )
    explore_delayed = TimerAction(period=30.0, actions=[explore_node])

    # map_saver exits after writing the map files (shutdown_on_complete=True).
    # Its OnProcessExit handler (below) starts Phase 2 instead of shutting down.
    map_saver = Node(
        package='warehouse_robot_bringup',
        executable='exploration_map_saver.py',
        name='exploration_map_saver',
        output='screen',
        parameters=[{
            'map_basename': map_basename,
            'use_sim_time': use_sim_time,
            'shutdown_on_complete': True,
        }],
    )

    # ── Phase 2: Pick-and-deliver ─────────────────────────────────────────────

    pallet_detector = Node(
        package='warehouse_perception',
        executable='pallet_detector',
        name='pallet_detector',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # pallet_name must match the model name in big_warehouse.sdf.
    pallet_handler = Node(
        package='warehouse_pallet_handler',
        executable='pallet_handler_node',
        name='pallet_handler',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'pallet_name': 'entrance_pallet',
            'world_name': 'big_warehouse',
        }],
    )

    mission_node = Node(
        package='warehouse_task_manager',
        executable='mission',
        name='mission',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )
    # Small delay: give pallet_detector and pallet_handler a moment to
    # start publishing before the mission node calls waitUntilNav2Active.
    mission_delayed = TimerAction(period=5.0, actions=[mission_node])

    # When the mission finishes, shut the whole stack down cleanly.
    on_mission_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=mission_node,
            on_exit=[Shutdown(reason='mission complete')],
        )
    )

    # Transition from Phase 1 to Phase 2 when the map_saver exits.
    on_saver_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=map_saver,
            on_exit=[
                pallet_detector,
                pallet_handler,
                mission_delayed,
                on_mission_exit,
            ],
        )
    )

    # Gate everything on cleanup finishing so stale processes can't race
    # the newly-spawned ones.
    after_cleanup = RegisterEventHandler(
        OnProcessExit(
            target_action=cleanup,
            on_exit=[
                sim,
                slam,
                rviz_inc,
                nav2_delayed,
                lift_forks_delayed,
                seed_spin_delayed,
                explore_delayed,
                map_saver,
                on_saver_exit,
            ],
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'map_basename', default_value=_default_map_basename(),
            description='Absolute basename for the explored-map files (no extension)',
        ),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='Launch RViz during the mission',
        ),
        cleanup,
        after_cleanup,
    ])
