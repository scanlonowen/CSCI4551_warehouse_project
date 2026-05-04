# Copyright (c) 2018 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_dir = get_package_share_directory('aws_robomaker_small_warehouse_world')
    ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Set model/resource path so gz sim can find the warehouse models
    models_path = os.path.join(pkg_dir, 'models')
    worlds_path = os.path.join(pkg_dir, 'worlds')

    world = LaunchConfiguration('world')

    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(pkg_dir, 'worlds', 'big_warehouse', 'big_warehouse.sdf'),
        description='Full path to world file to load')

    # Prepend model paths to GZ_SIM_RESOURCE_PATH
    gz_resource_env = SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        ':'.join([
            models_path,
            worlds_path,
            os.environ.get('GZ_SIM_RESOURCE_PATH', '')
        ])
    )

    # Launch Gazebo Harmonic via ros_gz_sim
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': ['-r ', world],
            'on_exit_shutdown': 'true',
        }.items()
    )

    return LaunchDescription([
        declare_world_cmd,
        gz_resource_env,
        gz_sim,
    ])
