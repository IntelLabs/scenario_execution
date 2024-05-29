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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetLaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():

    arm_sim_scenario_dir = get_package_share_directory('arm_sim_scenario')
    scenario_execution_dir = get_package_share_directory('scenario_execution_ros')
    # gazebo_tf_publisher_dir = get_package_share_directory('gazebo_tf_publisher')
    tf_to_pose_publisher_dir = get_package_share_directory('tf_to_pose_publisher')

    scenario = LaunchConfiguration('scenario')
    scenario_execution = LaunchConfiguration('scenario_execution')
    arg_scenario = DeclareLaunchArgument('scenario',
                                         description='Scenario file to execute')
    arg_scenario_execution = DeclareLaunchArgument(
        'scenario_execution', default_value='True',
        description='Wether to execute scenario execution')

    ignition = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution([arm_sim_scenario_dir, 'launch', 'ignition_launch.py'])]),
    )

    moveit_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution([arm_sim_scenario_dir, 'launch', 'moveit_launch.py'])]),
    )

    # groundtruth_publisher = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(os.path.join(gazebo_tf_publisher_dir, 'launch', 'gazebo_tf_publisher_launch.py')),
    #     launch_arguments=[
    #         ('ign_pose_topic', ['/world/', world, '/dynamic_pose/info']),
    #     ]
    # )

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
        arg_scenario_execution
    ])
    ld.add_action(ignition)
    ld.add_action(moveit_bringup)
    ld.add_action(scenario_exec)
    ld.add_action(tf_to_pose)
    return ld
