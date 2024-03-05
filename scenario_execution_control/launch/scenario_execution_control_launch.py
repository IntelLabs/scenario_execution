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

import launch
import launch_ros.actions


def generate_launch_description():
    ld = launch.LaunchDescription([

        launch.actions.DeclareLaunchArgument(
            'scenario_dir', description='Directory containing osc2 scenario files'),

        launch_ros.actions.Node(
            package='scenario_execution_control',
            executable='scenario_execution_control',
            name='scenario_execution_control',
            output='screen',
            emulate_tty='True',
            on_exit=launch.actions.Shutdown(),
        ),
        launch_ros.actions.Node(
            package='scenario_execution_control',
            executable='scenario_list_publisher',
            name='scenario_list_publisher',
            output='screen',
            on_exit=launch.actions.Shutdown(),
            parameters=[
                {'directory': launch.substitutions.LaunchConfiguration('scenario_dir')},
            ]
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
