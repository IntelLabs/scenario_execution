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
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


ARGUMENTS = [
    DeclareLaunchArgument('robot_name', default_value='robot2',
                          description='name of robot'),
]


def generate_launch_description():

    # Launch configurations
    robot_name = LaunchConfiguration('robot_name')

    cmd_vel_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='cmd_vel_bridge',
        output='screen',
        parameters=[{
            'use_sim_time': True
        }],
        arguments=[
            [robot_name,
             '/cmd_vel' + '@geometry_msgs/msg/Twist' + '[ignition.msgs.Twist'],
            ['/model/', robot_name, '/cmd_vel' +
             '@geometry_msgs/msg/Twist' +
             ']ignition.msgs.Twist']
        ],
        remappings=[
            (['/model/', robot_name, '/cmd_vel'], [robot_name, '/cmd_vel'])
        ]
    )

    # Define LaunchDescription variable
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(cmd_vel_bridge)
    return ld
