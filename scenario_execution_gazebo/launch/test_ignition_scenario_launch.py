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


import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():

    # Directories
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    scenario_execution_gazebo_dir = get_package_share_directory('scenario_execution_gazebo')
    scenario_execution_dir = get_package_share_directory('scenario_execution')

    turtlebot4_navigation_dir = get_package_share_directory('turtlebot4_navigation')

    headless = LaunchConfiguration('headless')
    scenario = LaunchConfiguration('scenario')
    execute_scenario = LaunchConfiguration('execute_scenario')
    spawn = LaunchConfiguration('spawn')

    # Generate launch description
    ld = LaunchDescription([

        DeclareLaunchArgument('headless', default_value='False',
                              description='Whether to execute simulation gui'),
        DeclareLaunchArgument('spawn', default_value='True', description='spawn robot?'),
        DeclareLaunchArgument('scenario', description='Scenario file to execute'),
        DeclareLaunchArgument('execute_scenario', default_value='True',
                              description='execute scenario?'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                nav2_bringup_dir, 'launch', 'bringup_launch.py')),
            launch_arguments={
                'map': PathJoinSubstitution([turtlebot4_navigation_dir, 'maps', 'maze.yaml']),
                'use_sim_time': 'true',
                'use_composition': 'False',
                'autostart': 'True',
                'params_file': PathJoinSubstitution([scenario_execution_gazebo_dir, 'params', 'nav2_params.yaml']), }.items()),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [scenario_execution_gazebo_dir, 'launch', 'ignition.launch.py'])),
            launch_arguments=[
                ('world', 'maze'),
                ('headless', headless),
            ]
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [scenario_execution_gazebo_dir, 'launch', 'turtlebot4_spawn.launch.py'])),
            condition=IfCondition(spawn),
            launch_arguments=[
                ('x', '0'),
                ('y', '0'),
                ('z', '0'),
                ('yaw', '0')]
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(scenario_execution_dir, 'launch', 'scenario_launch.py')),
            condition=IfCondition(execute_scenario),
            launch_arguments={'scenario': scenario,
                              'scenario_status': 'False',
                              'verbose': 'True'}.items()),

    ])
    return ld
