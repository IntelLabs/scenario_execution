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

from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    test_param = LaunchConfiguration('test_param')
    test_path = LaunchConfiguration('test_path')
    timeout = LaunchConfiguration('timeout')

    return LaunchDescription([
        DeclareLaunchArgument('test_param', description='test parameter'),
        DeclareLaunchArgument('test_path', description='Test path parameter'),
        DeclareLaunchArgument('timeout', description='Timeout', default_value='5'),

        Node(
            # condition=IfCondition(scenario_status),
            package='test_scenario_execution_ros',
            executable='test_scenario_execution_ros',
            name='test_scenario_execution_ros',
            parameters=[{
                'test_param': test_param,
                'test_path': test_path,
                'timeout': timeout,
            }],
            output='screen'
        ),
    ])
