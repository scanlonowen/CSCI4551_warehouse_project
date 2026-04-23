"""
Launch file: Warehouse + Forklift simulation.
Starts:
  1. Gazebo Harmonic with the no-roof warehouse world
  2. Robot state publisher (URDF -> TF)
  3. Spawns the forklift model into the warehouse
  4. ros_gz_bridge for clock + sensors
  5. ros2_control controllers (diff drive + fork + joint state broadcaster)
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    ExecuteProcess,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    # Package paths
    bringup_pkg = get_package_share_directory('warehouse_robot_bringup')
    warehouse_pkg = get_package_share_directory('aws_robomaker_small_warehouse_world')

    # World file
    world_file = os.path.join(
        warehouse_pkg, 'worlds', 'no_roof_small_warehouse', 'no_roof_small_warehouse.world'
    )

    # URDF via xacro
    urdf_file = os.path.join(bringup_pkg, 'urdf', 'forklift.urdf.xacro')
    robot_description = ParameterValue(
        Command(['xacro ', urdf_file]),
        value_type=str,
    )

    # Controller config
    controller_config = os.path.join(bringup_pkg, 'config', 'forklift_controllers.yaml')

    # Spawn position — open aisle in the warehouse
    # NOTE: (0,0) is blocked by WallB_01. This position is in a clear aisle.
    spawn_x = LaunchConfiguration('spawn_x', default='-1.5')
    spawn_y = LaunchConfiguration('spawn_y', default='-5.0')
    spawn_z = LaunchConfiguration('spawn_z', default='0.1')
    spawn_yaw = LaunchConfiguration('spawn_yaw', default='1.57')

    # ── 1. Gazebo Harmonic ─────────────────────────────────────────────
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch', 'gz_sim.launch.py',
            )
        ),
        launch_arguments={
            'gz_args': ['-r ', world_file],
            'on_exit_shutdown': 'true',
        }.items(),
    )

    # ── 2. Robot State Publisher ───────────────────────────────────────
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }],
        output='screen',
    )

    # ── 3. Spawn forklift into Gazebo ─────────────────────────────────
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'forklift',
            '-topic', 'robot_description',
            '-x', spawn_x,
            '-y', spawn_y,
            '-z', spawn_z,
            '-Y', spawn_yaw,
        ],
        output='screen',
    )

    # ── 4. ros_gz_bridge: clock + sensors ─────────────────────────────
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/lidar@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/camera@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
        ],
        remappings=[
            ('/lidar', '/scan'),
            ('/camera', '/camera/image_raw'),
            ('/camera_info', '/camera/camera_info'),
        ],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # ── 5. cmd_vel relay: /cmd_vel (Twist) → /diff_cont/cmd_vel (TwistStamped)
    # Jazzy's diff_drive_controller only accepts TwistStamped, but
    # teleop_twist_keyboard and Nav2 publish plain Twist on /cmd_vel.
    # This node converts between the two.
    cmd_vel_relay = ExecuteProcess(
        cmd=[
            'python3',
            os.path.join(bringup_pkg, 'scripts', 'twist_to_stamped.py'),
        ],
        output='screen',
    )

    # ── 6. ros2_control: spawn controllers ────────────────────────────
    # Joint state broadcaster (must start first)
    joint_broad_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_broad', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    # Diff drive controller
    diff_cont_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_cont', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    # Fork position controller
    fork_ctrl_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['fork_position_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    # Delay controller spawning until joint_broad is up
    delayed_diff = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_broad_spawner,
            on_exit=[diff_cont_spawner],
        )
    )

    delayed_fork = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_broad_spawner,
            on_exit=[fork_ctrl_spawner],
        )
    )

    # ── Launch arguments ──────────────────────────────────────────────
    declare_spawn_x = DeclareLaunchArgument('spawn_x', default_value='-1.5')
    declare_spawn_y = DeclareLaunchArgument('spawn_y', default_value='-5.0')
    declare_spawn_z = DeclareLaunchArgument('spawn_z', default_value='0.1')
    declare_spawn_yaw = DeclareLaunchArgument('spawn_yaw', default_value='1.57')

    return LaunchDescription([
        declare_spawn_x,
        declare_spawn_y,
        declare_spawn_z,
        declare_spawn_yaw,
        gazebo,
        robot_state_publisher,
        # Delay spawn slightly to let Gazebo initialize
        TimerAction(period=4.0, actions=[spawn_robot]),
        bridge,
        cmd_vel_relay,
        # Controllers start after a delay
        TimerAction(period=8.0, actions=[joint_broad_spawner]),
        delayed_diff,
        delayed_fork,
    ])
