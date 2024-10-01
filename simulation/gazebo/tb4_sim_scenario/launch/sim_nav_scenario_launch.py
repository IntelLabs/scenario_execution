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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():

    tb4_sim_scenario_dir = get_package_share_directory('tb4_sim_scenario')
    scenario_execution_dir = get_package_share_directory('scenario_execution_ros')
    gazebo_tf_publisher_dir = get_package_share_directory('gazebo_tf_publisher')
    tf_to_pose_publisher_dir = get_package_share_directory('tf_to_pose_publisher')

    scenario = LaunchConfiguration('scenario')
    scenario_execution = LaunchConfiguration('scenario_execution')
    arg_scenario = DeclareLaunchArgument('scenario',
                                         description='Scenario file to execute')
    arg_scenario_execution = DeclareLaunchArgument(
        'scenario_execution', default_value='true',
        choices=['true', 'false'],
        description='Whether to execute scenario execution')
    world_name = LaunchConfiguration('world_name')
    arg_world_name = DeclareLaunchArgument('world_name', default_value='default',
                                           description='Name of Simulation World')
    map_conf = LaunchConfiguration('map')
    arg_map = DeclareLaunchArgument('map', default_value=os.path.join(tb4_sim_scenario_dir, 'maps', 'maze.yaml'),
                                    description='Full path to map yaml file to load')

    simulation = GroupAction(
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([PathJoinSubstitution([tb4_sim_scenario_dir, 'launch', 'ignition_launch.py'])]),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([PathJoinSubstitution([tb4_sim_scenario_dir, 'launch', 'ignition_robot_launch.py'])])
            )]
    )

    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution([tb4_sim_scenario_dir, 'launch', 'nav2_launch.py'])]),
        launch_arguments={
            'use_sim_time': 'true',
            'map_yaml': map_conf,
        }.items()
    )

    groundtruth_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_tf_publisher_dir, 'launch', 'gazebo_tf_publisher_launch.py')),
        launch_arguments=[
            ('gz_pose_topic', ['/world/', world_name, '/dynamic_pose/info']),
        ]
    )

    scenario_exec = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution([scenario_execution_dir, 'launch', 'scenario_launch.py'])]),
        condition=IfCondition(scenario_execution),
        launch_arguments=[
            ('scenario', scenario),
        ]
    )

    tf_to_pose = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution([tf_to_pose_publisher_dir, 'launch', 'tf_to_pose_launch.py'])]),
    )

    ld = LaunchDescription([
        arg_scenario,
        arg_scenario_execution,
        arg_world_name,
        arg_map
    ])

    for pose_element in ['x', 'y', 'z', 'yaw']:
        ld.add_action(DeclareLaunchArgument(pose_element, default_value='0.0', description=f'{pose_element} component of the robot pose.'))
    ld.add_action(simulation)
    ld.add_action(nav2_bringup)
    ld.add_action(groundtruth_publisher)
    ld.add_action(scenario_exec)
    ld.add_action(tf_to_pose)
    return ld
