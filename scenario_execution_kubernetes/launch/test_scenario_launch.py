#!/usr/bin/env python3
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

""" Gazebo Test Scenario Launch """

import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

# Rviz requires US locale to correctly display the wheels
os.environ['LC_NUMERIC'] = 'en_US.UTF-8'


def generate_launch_description():
    """ generate launch description  """
    ignition_assets_dir = get_package_share_directory('ignition_assets')

    scenario = LaunchConfiguration('scenario')

    ld = LaunchDescription([

        DeclareLaunchArgument(
            'scenario',
            description='Scenario file to execute'),

        Node(
            package='scenario_execution',
            executable='scenario_execution',
            name='scenario_execution',
            output='screen',
            parameters=[{'use_sim_time': True}],
            arguments=[scenario]),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ignition_assets_dir, 'launch', 'warehouse1_launch.py'))),
    ])
    return ld
