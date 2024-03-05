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

# The main goal of this launch file is the visualization of the tf output topic in rviz2
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace

ARGUMENTS = [
    DeclareLaunchArgument('namespace', default_value='',
                          description='Robot namespace (without leading /)'),

    DeclareLaunchArgument('in_topic_scan', default_value='scan_sim',
                          description='Whether to execute simulation gui'),

    DeclareLaunchArgument('out_topic_scan', default_value='scan',
                          description='Whether to execute simulation gui'),

    DeclareLaunchArgument('drop_percentage', default_value='0.0',
                          description='Amount of dropped scan points in %'),

    DeclareLaunchArgument('std_dev_noise', default_value='0.0',
                          description='Whether to execute simulation gui'),
]


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    in_topic_scan = LaunchConfiguration("in_topic_scan")
    out_topic_scan = LaunchConfiguration("out_topic_scan")
    drop_percentage = LaunchConfiguration("drop_percentage")
    std_dev_noise = LaunchConfiguration("std_dev_noise")

    modification_group_action = GroupAction(
        [
            PushRosNamespace(namespace),
            Node(
                package="message_modification",
                name="laserscan_modification",
                executable="laserscan_modification",
                remappings=[
                    ("in", in_topic_scan),
                    ("out", out_topic_scan),
                ],
                parameters=[
                    {"random_drop_percentage": drop_percentage},
                    {"gaussian_noise_std_deviation": std_dev_noise},
                ],
            ),
        ]
    )
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(modification_group_action)
    return ld
