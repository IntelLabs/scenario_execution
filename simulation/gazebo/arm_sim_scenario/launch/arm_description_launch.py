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
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, PathJoinSubstitution
from launch.substitutions.launch_configuration import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue

ARGUMENTS = [
    DeclareLaunchArgument('ros2_control_hardware_type', default_value='mock_components',
                          choices=['ignition', 'mock_components'],
                          description='ROS2 control hardware interface type to use for the launch file'),
    DeclareLaunchArgument('use_sim_time', default_value='false',
                          choices=['true', 'false'],
                          description='use_sim_time'),
    DeclareLaunchArgument('virtual_joint_child_name', default_value='panda_link0',
                          description='arm base_link name'),
    DeclareLaunchArgument('virtual_joint_parent_frame', default_value='world',
                          description='virtual_joint_parent_frame name to which arm is attached to'),
    DeclareLaunchArgument('use_rviz', default_value='false',
                          choices=['true', 'false'],
                          description='launches RViz if set to `true`.'),
    DeclareLaunchArgument('rviz_config',
                          default_value=PathJoinSubstitution([get_package_share_directory('arm_sim_scenario'),
                                                              'config',
                                                              'arm.rviz',
                                                              ]),
                          description='file path to the config file RViz should load.',),
]


def generate_launch_description():
    pkg_moveit_resources_panda_moveit_config = get_package_share_directory('moveit_resources_panda_moveit_config')
    xacro_file = PathJoinSubstitution([pkg_moveit_resources_panda_moveit_config,
                                       'config',
                                       'panda.urdf.xacro'])
    ros2_control_hardware_type = LaunchConfiguration('ros2_control_hardware_type')
    virtual_joint_child_name = LaunchConfiguration('virtual_joint_child_name')
    virtual_joint_parent_frame = LaunchConfiguration('virtual_joint_parent_frame')
    rviz_config = LaunchConfiguration('rviz_config')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
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

    rviz_node = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=[
            '-d', rviz_config,
        ],
        parameters=[{
            'use_sim_time': use_sim_time,
        }],
        output={'both': 'log'},
    )

    # Define LaunchDescription variable
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(robot_state_publisher)
    ld.add_action(static_tf_node)
    ld.add_action(rviz_node)
    return ld
