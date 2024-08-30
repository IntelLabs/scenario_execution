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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


ARGUMENTS = [
    DeclareLaunchArgument('robot_model', default_value='wx200',
                          choices=['wx200'],
                          description='model type of the Interbotix Arm'),
    DeclareLaunchArgument('robot_name', default_value='wx200',
                          description='Robot name'),
    DeclareLaunchArgument('use_sim_time', default_value='true',
                          choices=['true', 'false'],
                          description='use_sim_time')

]


def generate_launch_description():

    robot_name = LaunchConfiguration('robot_name')
    use_sim_time = LaunchConfiguration('use_sim_time')

    spawn_robot_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'spawn_wx200',
                   '-x', '0.0',
                   '-y', '0.0',
                   '-z', '0.0',
                   '-Y', '0.0',
                   '-topic', 'wx200/robot_description'],
        output='screen'
    )

    spawn_joint_state_broadcaster_node = Node(
        name='joint_state_broadcaster_spawner',
        package='controller_manager',
        executable='spawner',
        namespace=robot_name,
        arguments=[
            '-c',
            'controller_manager',
            'joint_state_broadcaster',
        ],
        parameters=[{
            'use_sim_time': use_sim_time,
        }],
    )

    spawn_arm_controller_node = Node(
        name='arm_controller_spawner',
        package='controller_manager',
        executable='spawner',
        namespace=robot_name,
        arguments=[
            '-c',
            'controller_manager',
            'arm_controller',
        ],
        parameters=[{
            'use_sim_time': use_sim_time,
        }]
    )

    spawn_gripper_controller_node = Node(
        name='gripper_controller_spawner',
        package='controller_manager',
        executable='spawner',
        namespace=robot_name,
        arguments=[
            '-c',
            'controller_manager',
            'gripper_controller',
        ],
        parameters=[{
            'use_sim_time': use_sim_time,
        }]
    )

    load_joint_state_broadcaster_event = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_robot_node,
            on_exit=[spawn_joint_state_broadcaster_node]
        )
    )

    load_arm_controller_event = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_joint_state_broadcaster_node,
            on_exit=[spawn_arm_controller_node]
        )
    )

    load_gripper_controller_event = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_joint_state_broadcaster_node,
            on_exit=[spawn_gripper_controller_node]
        )
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(spawn_robot_node)
    ld.add_action(load_joint_state_broadcaster_event)
    ld.add_action(load_arm_controller_event)
    ld.add_action(load_gripper_controller_event)
    return ld
