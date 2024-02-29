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
from launch_ros.actions import Node, PushRosNamespace
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    """ generate launch description  """
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    scenario_execution_gazebo_dir = get_package_share_directory('scenario_execution_gazebo')

    ld = LaunchDescription([

        GroupAction([
            PushRosNamespace(namespace=['/tb3']),
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                output='screen',
                parameters=[{'use_sim_time': True}],
                arguments=[os.path.join(nav2_bringup_dir, 'urdf', 'turtlebot3_waffle.urdf')],
                remappings=[
                        ('/tf', 'tf'),
                        ('/tf_static', 'tf_static'),
                        ('/joint_states', 'joint_states')],),
        ]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')),
            launch_arguments={'namespace': 'tb3',
                              'use_namespace': 'true',
                              'slam': 'False',
                              'map': os.path.join(nav2_bringup_dir, 'maps', 'turtlebot3_world.yaml'),
                              'use_sim_time': 'True',
                              'params_file': os.path.join(scenario_execution_gazebo_dir, 'params', 'nav2_params.yaml'),
                              'autostart': 'True'}.items()),
    ])
    return ld
