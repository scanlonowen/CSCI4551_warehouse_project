"""
Stage 5.5: Autonomous frontier-based exploration.

Brings up:
  1. warehouse_simulation.launch.py  -> Gazebo + forklift + bridges + controllers
  2. slam.launch.py                   -> slam_toolbox online async (publishes
                                         /map and the map -> odom TF)
  3. nav2_bringup/navigation_launch.py with config/nav2_params_exploration.yaml
     -> controller / planner / behavior / smoother / bt_navigator /
        velocity_smoother / collision_monitor / lifecycle_manager
        (NO amcl, NO map_server -- slam_toolbox owns those)
  4. RViz with the forklift config
  5. explore_lite (m-explore-ros2 fork) -> picks frontier goals and sends
        them to Nav2's NavigateToPose action
  6. exploration_map_saver.py -> watches /explore/status; when exploration
        completes, calls /slam_toolbox/save_map + /slam_toolbox/serialize_map
        with basename
            src/warehouse_robot_bringup/maps/warehouse_explored
        (resolved to an absolute path on the user's host workspace), then
        shuts the stack down cleanly via OnProcessExit -> Shutdown.

Usage:
    ros2 launch warehouse_robot_bringup explore.launch.py

Optional args:
    map_basename:= absolute basename for the saved map files
                   (default: ~/warehouse_ws/src/warehouse_robot_bringup/maps/
                             warehouse_explored)
    keep_alive:=true  -> don't auto-shutdown after the map is saved
                         (handy for inspecting RViz post-exploration)
    rviz:=false       -> skip the RViz include
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
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def _default_map_basename() -> str:
    """Resolve the on-disk source-tree maps dir for the bringup pkg.

    We deliberately walk back from the installed share dir so the map files
    are written into the *source tree* (so they're visible to git and to the
    Stage 3 navigation.launch.py without re-running colcon).
    """
    share = get_package_share_directory('warehouse_robot_bringup')
    # share = .../install/warehouse_robot_bringup/share/warehouse_robot_bringup
    # walk to the workspace root, then dive into src/...
    ws_root = os.path.abspath(os.path.join(share, '..', '..', '..', '..'))
    candidate = os.path.join(
        ws_root, 'src', 'warehouse_robot_bringup', 'maps', 'warehouse_explored',
    )
    return candidate


def generate_launch_description():
    bringup_pkg = get_package_share_directory('warehouse_robot_bringup')
    nav2_pkg = get_package_share_directory('nav2_bringup')

    nav2_params = os.path.join(bringup_pkg, 'config', 'nav2_params_exploration.yaml')
    explore_params = os.path.join(bringup_pkg, 'config', 'explore_lite.yaml')

    # 0) Preflight: kill any stragglers from a previous run. launch_ros's
    # Shutdown doesn't reliably cascade to all ExecuteProcess children
    # (gz sim, scan_filter, twist_to_stamped) or to nav2 lifecycle nodes,
    # and leaked parameter_bridge processes each republish /clock --
    # multiple /clock streams cause TF buffers to thrash with "jump back
    # in time" warnings and the costmap drops every scan.
    cleanup = ExecuteProcess(
        cmd=[os.path.join(bringup_pkg, 'scripts', 'cleanup_stale_procs.sh')],
        output='screen',
    )

    map_basename = LaunchConfiguration(
        'map_basename', default=_default_map_basename(),
    )
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    keep_alive = LaunchConfiguration('keep_alive', default='false')
    rviz = LaunchConfiguration('rviz', default='true')

    # 1) Gazebo + forklift + controllers + bridges
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_pkg, 'launch', 'warehouse_simulation.launch.py')
        ),
    )

    # 2) slam_toolbox online async
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_pkg, 'launch', 'slam.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    # 3) RViz with the forklift config (optional)
    rviz_inc = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_pkg, 'launch', 'view_robot.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
        condition=IfCondition(rviz),
    )

    # 4) Nav2 nodes WITHOUT amcl / map_server. nav2_bringup's
    # navigation_launch.py is exactly that: it spins up controller_server,
    # planner_server, behavior_server, smoother_server, bt_navigator,
    # waypoint_follower, velocity_smoother, collision_monitor, and
    # lifecycle_manager_navigation. It does not start amcl or map_server
    # (those live in localization_launch.py / bringup_launch.py).
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

    # Delay Nav2 so SLAM has time to publish a /map and the map->odom TF
    # before nav2's costmaps come up.
    nav2_delayed = TimerAction(period=8.0, actions=[nav2])

    # 5a-pre) Spin the robot 360° in place to build an initial SLAM map
    # before explore_lite starts. Without this, explore_lite's first
    # frontier-search has only the forward arc visible and tends to pick a
    # frontier behind the robot, forcing an immediate 180°.
    # 0.5 rad/s for ~13 s = one ~6.5 rad rotation. With the wider
    # /scan_slam topic and a lower heading threshold in slam_toolbox,
    # this produces a denser seed map before frontier exploration begins.
    seed_spin = ExecuteProcess(
        cmd=[
            'ros2', 'topic', 'pub', '--rate', '10', '--times', '130',
            '/cmd_vel', 'geometry_msgs/msg/Twist',
            '{linear: {x: 0.0}, angular: {z: 0.5}}',
        ],
        output='screen',
    )
    seed_spin_delayed = TimerAction(period=15.0, actions=[seed_spin])

    # 5a) Lift the forks before exploration begins. The fork backing plate
    # (fork_base_link: 1.05 m wide, 1.22 m tall) sits in the LiDAR scan
    # plane (z=0.95 m above ground) at joint values below ~0.946 m.
    # Raising to 1.1 m puts the backing plate bottom at z=1.054 m, 0.10 m
    # above the scan plane, so it no longer generates self-hits that
    # contaminate the costmap and SLAM map.
    # Runs at t=12s — after fork_position_controller is active (t≈8s)
    # and before the seed_spin at t=15s.
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

    # 5b) Frontier explorer (explore_lite from m-explore-ros2). Wait for Nav2
    # to be alive before launching, otherwise its first NavigateToPose goal
    # gets rejected.
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
    # Start explore_lite right after the seed spin completes (~t=28s).
    # Earlier is fine because the global_costmap is rolling-window 60×60 m
    # and is centered on the robot from the moment Nav2 is up at t=8s.
    explore_delayed = TimerAction(period=30.0, actions=[explore_node])

    # 6) The save-on-complete hook. shutdown_on_complete is the inverse of
    # the keep_alive flag.
    shutdown_on_complete = PythonExpression(
        ["'", keep_alive, "' == 'false'"]
    )
    map_saver = Node(
        package='warehouse_robot_bringup',
        executable='exploration_map_saver.py',
        name='exploration_map_saver',
        output='screen',
        parameters=[{
            'map_basename': map_basename,
            'use_sim_time': use_sim_time,
            'shutdown_on_complete': shutdown_on_complete,
        }],
    )

    # When the map saver exits (which it does after writing the files, if
    # keep_alive is false), tear the rest of the stack down.
    on_saver_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=map_saver,
            on_exit=[Shutdown(reason='exploration complete, map saved')],
        )
    )

    # Gate the rest of the stack on cleanup exiting, so the preflight's
    # pkill calls can't race the new processes being started.
    after_cleanup = RegisterEventHandler(
        OnProcessExit(
            target_action=cleanup,
            on_exit=[
                sim,
                slam,
                rviz_inc,
                nav2_delayed,
                seed_spin_delayed,
                lift_forks_delayed,
                explore_delayed,
                map_saver,
                on_saver_exit,
            ],
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'map_basename', default_value=_default_map_basename(),
            description='Absolute basename for saved map files (no extension)',
        ),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use Gazebo simulation clock',
        ),
        DeclareLaunchArgument(
            'keep_alive', default_value='false',
            description='If true, keep the stack running after exploration '
                        'completes (for RViz inspection). Default is false: '
                        'auto-shutdown after the map is saved.',
        ),
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='Launch RViz with the forklift config',
        ),
        cleanup,
        after_cleanup,
    ])
