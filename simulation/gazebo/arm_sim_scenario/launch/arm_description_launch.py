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
from launch.substitutions import Command, PathJoinSubstitution
from launch.substitutions.launch_configuration import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

ARGUMENTS = [
    DeclareLaunchArgument('ros2_control_hardware_type', default_value='mock_components',
                          choices=['ignition', 'mock_components'],
                          description='ROS2 control hardware interface type to use for the launch file'),
    DeclareLaunchArgument('use_sim_time', default_value='true',
                          choices=['true', 'false'],
                          description='use_sim_time'),
    DeclareLaunchArgument('virtual_joint_child_name', default_value='panda_link0',
                          description='arm base_link name'),
    DeclareLaunchArgument('virtual_joint_parent_frame', default_value='world',
                          description='virtual_joint_parent_frame name to which arm is attached to'),
    DeclareLaunchArgument('urdf_pkg', default_value='arm_sim_scenario',
                          description='Package where URDF/Xacro file is located (file should be inside the config dir of pkg/config/robot_name.urdf.xacro)'),
    DeclareLaunchArgument('urdf', default_value='panda.urdf.xacro',
                          description='Name of URDF/Xacro file')
]


def generate_launch_description():

    pkg_urdf = FindPackageShare(LaunchConfiguration('urdf_pkg'))
    xacro_file = PathJoinSubstitution([pkg_urdf,
                                       'config',
                                       LaunchConfiguration('urdf')])
    ros2_control_hardware_type = LaunchConfiguration('ros2_control_hardware_type')
    virtual_joint_child_name = LaunchConfiguration('virtual_joint_child_name')
    virtual_joint_parent_frame = LaunchConfiguration('virtual_joint_parent_frame')
    use_sim_time = LaunchConfiguration('use_sim_time')

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[
            {'use_sim_time': use_sim_time},
            {'robot_description': ParameterValue(Command([
                'xacro', ' ', xacro_file, ' ',
                'ros2_control_hardware_type:=', ros2_control_hardware_type]), value_type=str)},
        ],
    )

    # Static TF
    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", virtual_joint_parent_frame, virtual_joint_child_name],
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}],
        output={'both': 'log'},
    )

    # Define LaunchDescription variable
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(robot_state_publisher)
    ld.add_action(joint_state_publisher)
    ld.add_action(static_tf_node)
    return ld
