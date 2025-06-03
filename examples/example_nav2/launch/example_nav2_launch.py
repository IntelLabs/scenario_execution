# Copyright (C) 2024 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing,
# software distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions
# and limitations under the License.
#
# SPDX-License-Identifier: Apache-2.0

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration


def generate_launch_description():

    example_nav2_dir = get_package_share_directory('example_nav2')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    scenario_execution_ros_dir = get_package_share_directory('scenario_execution_ros')

    scenario = LaunchConfiguration('scenario')

    return LaunchDescription([
        DeclareLaunchArgument('scenario', description='Scenario file to execute', default_value=PathJoinSubstitution([example_nav2_dir, 'scenarios', 'example_nav2.osc'])),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([PathJoinSubstitution([nav2_bringup_dir, 'launch', 'tb4_loopback_simulation.launch.py'])])
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([PathJoinSubstitution([scenario_execution_ros_dir, 'launch', 'scenario_launch.py'])]),
            launch_arguments={'scenario': scenario}.items()
        )
    ])
