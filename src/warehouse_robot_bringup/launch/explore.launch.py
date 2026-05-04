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

    # 5a-pre) Spin the robot 360° in place at t=13s to build the initial SLAM
    # map before explore_lite starts.  Without this, the robot is stationary
    # and SLAM's minimum_travel_distance stops it from processing more than
    # one scan, leaving explore_lite with almost no map to find frontiers in.
    # One full rotation (~6.3 s at 1 rad/s) covers all angles, then the robot
    # stops.  explore_lite is delayed to t=28s to start after the spin.
    # Slow rotation (~0.4 rad/s) over ~16 s = one full ~6.4 rad rotation.
    # Slow enough that SLAM and the obstacle_layer can keep up between scans.
    seed_spin = ExecuteProcess(
        cmd=[
            'ros2', 'topic', 'pub', '--rate', '10', '--times', '160',
            '/cmd_vel', 'geometry_msgs/msg/Twist',
            '{linear: {x: 0.0}, angular: {z: 0.4}}',
        ],
        output='screen',
    )
    seed_spin_delayed = TimerAction(period=13.0, actions=[seed_spin])

    # 5a) Lift the forks before exploration begins. The fork backing plate
    # (1.05 m wide, mounted 7 cm behind the LiDAR at z=0.15 m) sits inside
    # the LiDAR scan plane at the default joint position (0), creating a
    # fake wall in the costmap. Raising to 0.3 m puts the entire backing
    # plate and tines above z=0.254 m, clearing the scan plane entirely.
    # Runs at t=12s — after the fork_position_controller is active (t≈8s).
    lift_forks = ExecuteProcess(
        cmd=[
            'ros2', 'topic', 'pub', '--once',
            '/fork_position_controller/commands',
            'std_msgs/msg/Float64MultiArray',
            '{data: [0.3]}',
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
    explore_delayed = TimerAction(period=35.0, actions=[explore_node])

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
        sim,
        slam,
        rviz_inc,
        nav2_delayed,
        seed_spin_delayed,
        lift_forks_delayed,
        explore_delayed,
        map_saver,
        on_saver_exit,
    ])
