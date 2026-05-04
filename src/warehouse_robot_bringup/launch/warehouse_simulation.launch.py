"""
Launch file: Warehouse + Forklift simulation.
Starts:
  1. Gazebo Harmonic with the selected warehouse world (default: big_warehouse)
  2. Robot state publisher (URDF -> TF)
  3. Spawns the forklift model into the warehouse
  4. ros_gz_bridge for clock + sensors
  5. ros2_control controllers (diff drive + fork + joint state broadcaster)

Args:
  world:=big_warehouse | small_warehouse | no_roof_small_warehouse
  spawn_x, spawn_y, spawn_z, spawn_yaw  (override per-world spawn pose)
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    ExecuteProcess,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


# Map world name -> (worlds-subdir, filename) inside aws_robomaker_small_warehouse_world.
# Add new entries here when more worlds become available.
WORLD_FILES = {
    'big_warehouse': ('big_warehouse', 'big_warehouse.sdf'),
    'small_warehouse': ('small_warehouse', 'small_warehouse.world'),
    'no_roof_small_warehouse': ('no_roof_small_warehouse', 'no_roof_small_warehouse.world'),
}


def generate_launch_description():

    # Package paths
    bringup_pkg = get_package_share_directory('warehouse_robot_bringup')
    warehouse_pkg = get_package_share_directory('aws_robomaker_small_warehouse_world')

    # World selection: `world:=big_warehouse` (default) | `small_warehouse` | `no_roof_small_warehouse`
    world_name = LaunchConfiguration('world')
    declare_world = DeclareLaunchArgument(
        'world',
        default_value='big_warehouse',
        description=(
            'Warehouse world to load. One of: '
            + ', '.join(sorted(WORLD_FILES.keys()))
        ),
    )

    # Resolve world file path via PythonExpression so the LaunchConfiguration is
    # evaluated at launch time. Uses a dict literal embedded in the expression.
    world_files_repr = repr(WORLD_FILES)
    worlds_root = os.path.join(warehouse_pkg, 'worlds')
    world_file = PythonExpression([
        "'", worlds_root, "' + '/' + ",
        world_files_repr, "['", world_name, "'][0] + '/' + ",
        world_files_repr, "['", world_name, "'][1]",
    ])

    # Make the AWS warehouse models discoverable by gz sim regardless of which
    # world is selected (some worlds <include> models that live in this share dir).
    gz_resource_env = SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        ':'.join([
            os.path.join(warehouse_pkg, 'models'),
            os.path.join(warehouse_pkg, 'worlds'),
            os.environ.get('GZ_SIM_RESOURCE_PATH', ''),
        ]),
    )

    # URDF via xacro
    urdf_file = os.path.join(bringup_pkg, 'urdf', 'forklift.urdf.xacro')
    robot_description = ParameterValue(
        Command(['xacro ', urdf_file]),
        value_type=str,
    )

    # Controller config
    controller_config = os.path.join(bringup_pkg, 'config', 'forklift_controllers.yaml')

    # Spawn position.
    # Default targets the big_warehouse entrance bay center (0, -16) facing +Y.
    # Override via `spawn_x:= spawn_y:= spawn_yaw:=` for the small_warehouse
    # regression case (use e.g. -1.5, -5.0, 1.57 there).
    spawn_x = LaunchConfiguration('spawn_x', default='0.0')
    spawn_y = LaunchConfiguration('spawn_y', default='-16.0')
    spawn_z = LaunchConfiguration('spawn_z', default='0.1')
    spawn_yaw = LaunchConfiguration('spawn_yaw', default='1.5708')

    # ── 1. Gazebo Harmonic ─────────────────────────────────────────────
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch', 'gz_sim.launch.py',
            )
        ),
        launch_arguments={
            'gz_args': ['-r --headless-rendering ', world_file],
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
    # Bridge publishes /scan_raw (full 360°).  A laser_filters node below
    # strips the rear self-hit arc and republishes as /scan.
    # SLAM subscribes to /scan_raw (full 360° helps loop closure).
    # Nav2 costmap and collision_monitor subscribe to /scan (clean).
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
            ('/lidar', '/scan_raw'),
            ('/camera', '/camera/image_raw'),
            ('/camera_info', '/camera/camera_info'),
        ],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # ── 4b. Scan filter: remove forklift self-hits from rear arc ─────
    scan_filter = ExecuteProcess(
        cmd=[
            'python3',
            os.path.join(bringup_pkg, 'scripts', 'scan_filter_node.py'),
        ],
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
    declare_spawn_x = DeclareLaunchArgument('spawn_x', default_value='0.0')
    declare_spawn_y = DeclareLaunchArgument('spawn_y', default_value='-16.0')
    declare_spawn_z = DeclareLaunchArgument('spawn_z', default_value='0.1')
    declare_spawn_yaw = DeclareLaunchArgument('spawn_yaw', default_value='1.5708')

    return LaunchDescription([
        declare_world,
        declare_spawn_x,
        declare_spawn_y,
        declare_spawn_z,
        declare_spawn_yaw,
        gz_resource_env,
        gazebo,
        robot_state_publisher,
        # Delay spawn slightly to let Gazebo initialize
        TimerAction(period=4.0, actions=[spawn_robot]),
        bridge,
        scan_filter,
        cmd_vel_relay,
        # Controllers start after a delay
        TimerAction(period=8.0, actions=[joint_broad_spawner]),
        delayed_diff,
        delayed_fork,
    ])
