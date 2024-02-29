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
                          description='Robot namespace (including leading /)'),

    DeclareLaunchArgument('base_frame_id', default_value='base_link',
                          description='base frame id of the robot'),

    DeclareLaunchArgument('ground_truth_frame_id', default_value='turtlebot4_base_link_gt',
                          description='ground truth frame id of the robot'),

]


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    base_frame_id = LaunchConfiguration("base_frame_id")
    ground_truth_frame_id = LaunchConfiguration("ground_truth_frame_id")

    base_link_pose_pub = GroupAction(
        [
            PushRosNamespace(
                namespace=namespace
            ),
            Node(
                name='tf_to_pose_map_baselink',
                package='tf_to_pose_publisher',
                executable='tf_to_pose_publisher',
                output='screen',
                remappings=[('/tf', 'tf'),
                            ('/tf_static', 'tf_static'),
                            ('tf_as_pose', 'robot_pose_loc')],
                parameters=[{
                    'child_frame_id': base_frame_id,
                }]
            ),
        ]
    )

    base_link_ground_truth_pose_pub = GroupAction(
        [
            PushRosNamespace(
                namespace=namespace
            ),
            Node(
                name='tf_to_pose_map_baselink_gt',
                package='tf_to_pose_publisher',
                executable='tf_to_pose_publisher',
                output='screen',
                remappings=[('/tf', 'tf'),
                            ('/tf_static', 'tf_static'),
                            ('tf_as_pose', 'robot_pose_gt')],
                parameters=[{
                    'child_frame_id': ground_truth_frame_id,
                }]
            ),
        ]
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(base_link_pose_pub)
    ld.add_action(base_link_ground_truth_pose_pub)
    return ld
