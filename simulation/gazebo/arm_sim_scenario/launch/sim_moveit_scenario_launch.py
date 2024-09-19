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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


ARGUMENTS = [
    DeclareLaunchArgument('use_rviz', default_value='false',
                          choices=['true', 'false'],
                          description='launches RViz if set to `true`.'),
    DeclareLaunchArgument('ros2_control_hardware_type',
                          default_value='mock_components',
                          choices=['ignition', 'mock_components'],
                          description='ROS2 control hardware interface type to use for the launch file',
                          ),
    DeclareLaunchArgument('scenario',
                          default_value='',
                          description='Scenario file to execute',
                          ),
    DeclareLaunchArgument('scenario_execution', default_value='true',
                          choices=['true', 'false'],
                          description='Wether to execute scenario execution'),
]


def generate_launch_description():

    arm_sim_scenario_dir = get_package_share_directory('arm_sim_scenario')
    scenario_execution_dir = get_package_share_directory('scenario_execution_ros')

    use_rviz = LaunchConfiguration('use_rviz')
    ros2_control_hardware_type = LaunchConfiguration('ros2_control_hardware_type')
    scenario = LaunchConfiguration('scenario')
    scenario_execution = LaunchConfiguration('scenario_execution')

    moveit_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution([arm_sim_scenario_dir, 'launch', 'moveit_launch.py'])]),
        launch_arguments={
            'arg_ros2_control_hardware_type': ros2_control_hardware_type,
        }.items(),
    )

    arm_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution([arm_sim_scenario_dir, 'launch', 'arm_description_launch.py'])]),
        launch_arguments={
            'use_rviz': use_rviz,
            'arg_ros2_control_hardware_type': ros2_control_hardware_type,
        }.items()
    )

    controller_manager = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution([arm_sim_scenario_dir, 'launch', 'controller_manager_launch.py'])]),
        condition=LaunchConfigurationEquals(
            launch_configuration_name='arg_ros2_control_hardware_type',
            expected_value='mock_components'
        ),
    )

    scenario_exec = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution([scenario_execution_dir, 'launch', 'scenario_launch.py'])]),
        condition=IfCondition(scenario_execution),
        launch_arguments=[
            ('scenario', scenario),
        ]
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(moveit_bringup)
    ld.add_action(arm_description)
    ld.add_action(controller_manager)
    ld.add_action(scenario_exec)
    return ld
