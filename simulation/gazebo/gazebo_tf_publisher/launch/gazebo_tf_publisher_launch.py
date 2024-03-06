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
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    use_namespace = LaunchConfiguration("use_namespace")
    ign_pose_topic = LaunchConfiguration("ign_pose_topic")
    base_frame_id = LaunchConfiguration("base_frame_id")
    ld = LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_namespace",
                description="Execute all nodes/topics within /namespace namespace",
                default_value="False",
            ),
            DeclareLaunchArgument(
                "namespace",
                description="robot name if the robot is started in a ROS namespace",
                default_value="leo_sim",
            ),
            DeclareLaunchArgument(
                "ign_pose_topic",
                description="ignition topic to subscribe to",
            ),
            DeclareLaunchArgument(
                "base_frame_id",
                description="name of the frame_id of the robot",
                default_value="base_link",
            ),
            GroupAction(
                [
                    PushRosNamespace(
                        condition=IfCondition(use_namespace), namespace=["/", namespace]
                    ),
                    Node(
                        package="gazebo_tf_publisher",
                        name="gazebo_tf_publisher",
                        executable="gazebo_tf_publisher_node",
                        parameters=[
                            {"ign_pose_topic": ign_pose_topic},
                            {"base_frame_id": base_frame_id},
                        ],
                    ),
                ]
            ),
        ]
    )
    return ld
