import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    ExecuteProcess,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    # Package paths
    bringup_pkg = get_package_share_directory('warehouse_robot_bringup')

    # --- PORTABILITY FIX FOR THE GROUP ---
    models_path = os.path.join(get_package_share_directory('warehouse_robot_bringup'), 'models')

    if 'GZ_SIM_RESOURCE_PATH' in os.environ:
        os.environ['GZ_SIM_RESOURCE_PATH'] += os.pathsep + models_path
    else:
        os.environ['GZ_SIM_RESOURCE_PATH'] = models_path
    # -------------------------------------

    # # World file
    # world_file = 'empty.sdf'
    # World file
    world_file = os.path.join(bringup_pkg, 'worlds', 'warehouse_test.sdf')
    
    # URDF via xacro
    urdf_file = os.path.join(bringup_pkg, 'urdf', 'forklift.urdf.xacro')
    robot_description = ParameterValue(
        Command(['xacro ', urdf_file]),
        value_type=str,
    )

    # Pallet SDF Path
    pallet_sdf_file = os.path.join(models_path, 'euro_pallet', 'euro_pallet.sdf')

    # Spawn position (Forklift)
    spawn_x = LaunchConfiguration('spawn_x', default='0.0')
    spawn_y = LaunchConfiguration('spawn_y', default='0.0')
    spawn_z = LaunchConfiguration('spawn_z', default='0.2') # Bumped to 0.2 for safety
    spawn_yaw = LaunchConfiguration('spawn_yaw', default='0.0')

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

    # ── 3.5 Spawn Euro Pallet ─────────────────────────────────────────
    spawn_pallet = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'pallet_1',
            '-file', pallet_sdf_file,
            '-x', '2.0',  # 2 meters in front of the origin
            '-y', '0.0',
            '-z', '0.0', 
        ],
        output='screen',
    )

    # ── 4. ros_gz_bridge: clock + sensors ─────────────────────────────
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',  # Updated from /lidar
            '/camera@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/model/forklift/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
        ],
        remappings=[
            ('/camera', '/camera/image_raw'),
            ('/camera_info', '/camera/camera_info'),
        ],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # ── 5. cmd_vel relay ──────────────────────────────────────────────
    cmd_vel_relay = ExecuteProcess(
        cmd=[
            'python3',
            os.path.join(bringup_pkg, 'scripts', 'twist_to_stamped.py'),
        ],
        output='screen',
    )

    # ── 6. ros2_control: spawners ─────────────────────────────────────
    joint_broad_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_broad', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    diff_cont_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_cont', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    fork_ctrl_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['fork_position_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    # ── Launch arguments ──────────────────────────────────────────────
    declare_spawn_x = DeclareLaunchArgument('spawn_x', default_value='0.0')
    declare_spawn_y = DeclareLaunchArgument('spawn_y', default_value='0.0')
    declare_spawn_z = DeclareLaunchArgument('spawn_z', default_value='0.2')
    declare_spawn_yaw = DeclareLaunchArgument('spawn_yaw', default_value='0.0')

    return LaunchDescription([
        declare_spawn_x,
        declare_spawn_y,
        declare_spawn_z,
        declare_spawn_yaw,
        gazebo,
        robot_state_publisher,
        
        # Spawn the robot and the pallet simultaneously after 3 seconds
        TimerAction(period=3.0, actions=[spawn_robot, spawn_pallet]),
        
        bridge,
        cmd_vel_relay,

        TimerAction(period=8.0, actions=[
            joint_broad_spawner,
            diff_cont_spawner,
            fork_ctrl_spawner
        ]),
    ])