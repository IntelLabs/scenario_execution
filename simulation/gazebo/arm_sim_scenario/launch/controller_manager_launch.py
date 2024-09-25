# Copyright (C) 2024 Intel Corporation
# Copyright 2021 Clearpath Robotics, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# @author Roni Kreinin (rkreinin@clearpathrobotics.com)


from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions.launch_configuration import LaunchConfiguration
from launch.conditions import LaunchConfigurationNotEquals
from launch_ros.actions import Node
import os

ARGUMENTS = [
    DeclareLaunchArgument('arm_group_controller', default_value='panda_arm_controller',
                          description='arm_group_controller name'),
    DeclareLaunchArgument('gripper_group_controller', default_value='panda_hand_controller',
                          description='gripper_group_controller name'),
    DeclareLaunchArgument('ros2_control_hardware_type', default_value='mock_components',
                          choices=['ignition', 'mock_components'],
                          description='ROS2 control hardware interface type to use for the launch file'),
]


def generate_launch_description():

    pkg_moveit_resources_panda_moveit_config = get_package_share_directory('moveit_resources_panda_moveit_config')
    arm_group_controller = LaunchConfiguration('arm_group_controller')
    gripper_group_controller = LaunchConfiguration('gripper_group_controller')

    ros2_controllers_path = os.path.join(
        pkg_moveit_resources_panda_moveit_config,
        "config",
        "ros2_controllers.yaml",
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[ros2_controllers_path],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
        output="screen",
        condition=LaunchConfigurationNotEquals(
            "ros2_control_hardware_type", "ignition"
        ),
    )

    joint_state_broadcaster = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "joint_state_broadcaster",
        ],
        output="screen",
    )

    arm_controller_spawner = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            arm_group_controller,
        ],
        output="screen",
    )

    gripper_controller_spawner = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            gripper_group_controller
        ],
        output="screen",
    )

    # Define LaunchDescription variable
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(ros2_control_node)
    ld.add_action(joint_state_broadcaster)
    ld.add_action(arm_controller_spawner)
    ld.add_action(gripper_controller_spawner)
    return ld
