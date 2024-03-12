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
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():

    tb4_sim_scenario_dir = get_package_share_directory('tb4_sim_scenario')
    scenario_execution_control_dir = get_package_share_directory('scenario_execution_control')
    example_scenario_control_dir = get_package_share_directory('example_scenario_control')
    scenario_dir = LaunchConfiguration('scenario_dir')
    rviz_config = LaunchConfiguration('rviz_config')
    
    arg_scenario_dir = DeclareLaunchArgument('scenario_dir', default_value=os.path.join(
        example_scenario_control_dir, 'scenarios'), description='default directory of the scenarios')
    
    arg_rviz_config = DeclareLaunchArgument('rviz_config', default_value=PathJoinSubstitution(
        [example_scenario_control_dir, 'rviz', 'example_control.rviz']), description='default directory of the scenarios')
    
    tb4_sim_scenario = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution([tb4_sim_scenario_dir, 'launch', 'sim_nav_scenario_launch.py'])]),
        launch_arguments= {'scenario_execution' : 'False', 'scenario' : []}.items()
    )

    rviz = LaunchDescription([
        Node(
            package='rviz2',
            namespace='',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config]
        )
    ])

    scenario_execution_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(scenario_execution_control_dir, 'launch', 'scenario_execution_control_launch.py')),
        launch_arguments=[('scenario_dir', scenario_dir)]
    )

    ld = LaunchDescription([
        arg_scenario_dir,
        arg_rviz_config
    ])
    ld.add_action(scenario_execution_control)
    ld.add_action(tb4_sim_scenario)
    ld.add_action(rviz)
    return ld
