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
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import LaunchConfigurationEquals


ARGUMENTS = [
    DeclareLaunchArgument('use_rviz', default_value='false',
                          choices=['true', 'false'],
                          description='launches RViz if set to `true`.'),
    DeclareLaunchArgument('hardware_type',
                          default_value='fake',
                          choices=['actual', 'fake', 'gz_classic', 'ignition'],
                          description='configure robot_description to use actual, fake, or simulated hardware',
                          ),
    DeclareLaunchArgument('scenario',
                          default_value='',
                          description='Scenario file to execute',
                          ),
    DeclareLaunchArgument('scenario_execution', default_value='true',
                          choices=['true', 'false'],
                          description='Wether to execute scenario execution'),
    DeclareLaunchArgument('use_moveit', default_value='true',
                          choices=['true', 'false'],
                          description='Wether to launch moveit'),
]


def generate_launch_description():

    arm_sim_scenario_dir = get_package_share_directory('arm_sim_scenario')
    scenario_execution_dir = get_package_share_directory('scenario_execution_ros')
    use_rviz = LaunchConfiguration('use_rviz')
    hardware_type = LaunchConfiguration('hardware_type')
    scenario = LaunchConfiguration('scenario')
    scenario_execution = LaunchConfiguration('scenario_execution')
    use_moveit = LaunchConfiguration('use_moveit')

    ignition = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution([arm_sim_scenario_dir, 'launch', 'ignition_launch.py'])]),
        condition=LaunchConfigurationEquals(
            launch_configuration_name='hardware_type',
            expected_value='ignition'
        ),
    )

    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution([arm_sim_scenario_dir, 'launch', 'ignition_arm_launch.py'])]),
        condition=LaunchConfigurationEquals(
            launch_configuration_name='hardware_type',
            expected_value='ignition'
        ),
    )

    moveit_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution([arm_sim_scenario_dir, 'launch', 'moveit_launch.py'])]),
        condition=LaunchConfigurationEquals(
            launch_configuration_name='use_moveit',
            expected_value='true'
        ),
        launch_arguments={
            'hardware_type': hardware_type,
        }.items(),
    )

    arm_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution([arm_sim_scenario_dir, 'launch', 'arm_description_launch.py'])]),
        launch_arguments={
            'use_rviz': use_rviz,
            'hardware_type': hardware_type,
        }.items()
    )

    arm_controllers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution([arm_sim_scenario_dir, 'launch', 'arm_controller_launch.py'])]),
        launch_arguments={
            'hardware_type': hardware_type,
        }.items(),
        condition=LaunchConfigurationEquals(
            launch_configuration_name='hardware_type',
            expected_value='fake'
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
    ld.add_action(arm_controllers)
    ld.add_action(arm_description)
    ld.add_action(ignition)
    ld.add_action(robot)
    ld.add_action(moveit_bringup)
    ld.add_action(scenario_exec)
    return ld
