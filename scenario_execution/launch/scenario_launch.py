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
    scenario_status = LaunchConfiguration('scenario_status')
    output_dir = LaunchConfiguration('output_dir')

    return LaunchDescription([
        DeclareLaunchArgument('scenario', description='Scenario file to execute'),
        DeclareLaunchArgument('debug', description='Debug output', default_value='False'),
        DeclareLaunchArgument('live_tree', default_value='False',
                              description='output live state of scenario'),
        DeclareLaunchArgument('log_model', default_value='False',
                              description='log parsed model'),
        DeclareLaunchArgument('scenario_execution', default_value='True',
                              description='Wether to execute scenario execution'),
        DeclareLaunchArgument('scenario_status', default_value='False',
                              description='Wether to execute scenario status'),
        DeclareLaunchArgument('log_level', default_value='info',
                              description='Log level for scenario execution'),
        DeclareLaunchArgument('output_dir', description='Output directory', default_value=''),

        Node(
            package='scenario_execution',
            executable='scenario_execution',
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
                'scenario': scenario
            }],
            on_exit=Shutdown()),

        Node(
            condition=IfCondition(scenario_status),
            package='scenario_status',
            executable='scenario_status_node',
            name='scenario_status_node',
            parameters=[{
                'bt_snapshot_topic': '/bt_snapshot',
                'scenario_status_topic': '/scenario_status',
                'snapshot_srv_name': '/scenario_execution/snapshot_streams/open',

            }],
            output='screen'
        ),

        # Parse log for message on how to execute scenario separately
        LogInfo(
            condition=UnlessCondition(scenario_execution),
            msg=["Skipping: ros2 launch scenario_execution scenario_launch.py scenario:=",
                 scenario, ' log_level:="', log_level, '"']),
    ])
