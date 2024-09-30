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


import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, PathJoinSubstitution
from launch.substitutions.launch_configuration import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue

ARGUMENTS = [
    DeclareLaunchArgument('model', default_value='standard',
                          choices=['standard', 'lite'],
                          description='Turtlebot4 Model'),
    DeclareLaunchArgument('use_sim_time', default_value='false',
                          choices=['true', 'false'],
                          description='use_sim_time'),
    DeclareLaunchArgument('robot_name', default_value='turtlebot4',
                          description='Robot name'),
    DeclareLaunchArgument('namespace', default_value=LaunchConfiguration('robot_name'),
                          description='Robot namespace'),
    DeclareLaunchArgument(
        'control_config',
        default_value='irobot_create_control/config/control.yaml',
        description='Path to the control YAML file <package_name>/path/to/control.yaml)'
    )
]


def resolve_control_config_path(context):
    control_config_str = LaunchConfiguration('control_config').perform(context)
    try:
        package_name, relative_path = control_config_str.split('/', 1)
        package_share_path = get_package_share_directory(package_name)
        resolved_path = os.path.join(package_share_path, relative_path)
        return resolved_path
    except ValueError:
        raise RuntimeError(f"Invalid control_config file path format: '{control_config_str}'. "
                           f"Expected '<package_name>/path/to/control.yaml)'.")


def create_robot_state_publisher_node(context):
    control_config = resolve_control_config_path(context)
    pkg_tb4_sim_scenario = get_package_share_directory('tb4_sim_scenario')
    xacro_file = PathJoinSubstitution([pkg_tb4_sim_scenario,
                                       'urdf',
                                       'turtlebot4.urdf.xacro'])
    namespace = LaunchConfiguration('namespace')
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'robot_description': ParameterValue(Command([
                'xacro', ' ', xacro_file, ' ',
                'gazebo:=ignition', ' ',
                'namespace:=', namespace, ' ',
                'control_config:=', control_config]), value_type=str)},
        ],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ]
    )
    return [robot_state_publisher]


def generate_launch_description():

    robot_state_publisher = OpaqueFunction(function=create_robot_state_publisher_node)

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ]
    )

    # Define LaunchDescription variable
    ld = LaunchDescription(ARGUMENTS)
    # Add nodes to LaunchDescription
    ld.add_action(robot_state_publisher)
    ld.add_action(joint_state_publisher)
    return ld
