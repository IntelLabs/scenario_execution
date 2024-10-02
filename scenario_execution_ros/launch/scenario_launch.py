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

""" Scenario Launch """
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown, LogInfo
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():
    """ generate launch description  """
    scenario = LaunchConfiguration('scenario')
    debug = LaunchConfiguration('debug')
    live_tree = LaunchConfiguration('live_tree')
    log_model = LaunchConfiguration('log_model')
    log_level = LaunchConfiguration('log_level')
    scenario_execution = LaunchConfiguration('scenario_execution')
    output_dir = LaunchConfiguration('output_dir')
    scenario_parameter_file = LaunchConfiguration('scenario_parameter_file')

    return LaunchDescription([
        DeclareLaunchArgument('scenario', description='Scenario file to execute'),
        DeclareLaunchArgument('debug', description='Debug output', default_value='False'),
        DeclareLaunchArgument('live_tree', default_value='False',
                              description='output live state of scenario'),
        DeclareLaunchArgument('log_model', default_value='False',
                              description='log parsed model'),
        DeclareLaunchArgument('scenario_execution', default_value='True',
                              description='Wether to execute scenario execution'),
        DeclareLaunchArgument('log_level', default_value='info',
                              description='Log level for scenario execution'),
        DeclareLaunchArgument('output_dir', description='Output directory', default_value=''),
        DeclareLaunchArgument('scenario_parameter_file', description='Yaml file specifying scenario parameter overrides', default_value=''),

        Node(
            condition=IfCondition(scenario_execution),
            package='scenario_execution_ros',
            executable='scenario_execution_ros',
            name='scenario_execution',
            output='screen',
            additional_env={'PYTHONUNBUFFERED': '1'},
            arguments=['--ros-args', '--log-level', log_level],
            parameters=[{
                'use_sim_time': False,
                'debug': debug,
                'live_tree': live_tree,
                'log_model': log_model,
                'output_dir': output_dir,
                'scenario': scenario,
                'scenario_parameter_file': scenario_parameter_file
            }],
            on_exit=Shutdown()),

        # Parse log for message on how to execute scenario separately
        LogInfo(
            condition=UnlessCondition(scenario_execution),
            msg=["Skipping: ros2 launch scenario_execution_ros scenario_launch.py scenario:=",
                 scenario, ' log_model:=', log_model, ' live_tree:=', live_tree, ' debug:=', debug, ' output_dir:=', output_dir, ' scenario_parameter_file:=', scenario_parameter_file]),
    ])
