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
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, PathJoinSubstitution
from launch.substitutions.launch_configuration import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue

ARGUMENTS = [
    DeclareLaunchArgument('robot_model', default_value='wx200',
                          choices=['wx200'],
                          description='robot_model type of the Interbotix Arm'),
    DeclareLaunchArgument('robot_name', default_value=LaunchConfiguration('robot_model'),  # namespace
                          description='Robot name'),
    DeclareLaunchArgument('use_rviz', default_value='true',
                          choices=['true', 'false'],
                          description='launches RViz if set to `true`.'),
    DeclareLaunchArgument('rviz_config',
                          default_value=PathJoinSubstitution([get_package_share_directory('arm_sim_scenario'),
                                                              'rviz',
                                                              'xsarm_description.rviz',
                                                              ]),
                          description='file path to the config file RViz should load.'),
    DeclareLaunchArgument('use_sim_time', default_value='true',
                          choices=['true', 'false'],
                          description='use_sim_time'),
    DeclareLaunchArgument('use_joint_pub',
                          default_value='false',
                          choices=('true', 'false'),
                          description='launches the joint_state_publisher node.',
                          ),
    DeclareLaunchArgument('use_joint_pub_gui',
                          default_value='false',
                          choices=('true', 'false'),
                          description='launches the joint_state_publisher GUI.',
                          ),
    DeclareLaunchArgument('hardware_type',
                          default_value='fake',
                          choices=['actual', 'fake', 'gz_classic', 'ignition'],
                          description='configure robot_description to use actual, fake, or simulated hardware',
                          )
]


def generate_launch_description():
    pkg_arm_sim_scenario = get_package_share_directory('arm_sim_scenario')
    xacro_file = PathJoinSubstitution([pkg_arm_sim_scenario,
                                       'urdf',
                                       'wx200.urdf.xacro'])
    robot_model = LaunchConfiguration('robot_model')
    robot_name = LaunchConfiguration('robot_name')
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config = LaunchConfiguration('rviz_config')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_joint_pub = LaunchConfiguration('use_joint_pub')
    use_joint_pub_gui = LaunchConfiguration('use_joint_pub_gui')
    hardware_type = LaunchConfiguration('hardware_type')

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'robot_description': ParameterValue(Command([
                'xacro ', xacro_file, ' ',
                'robot_model:=', robot_model, ' ',
                'robot_name:=', robot_name, ' ',
                'use_world_frame:=', 'true '
                'hardware_type:=', hardware_type]), value_type=str)},
        ],
        namespace=robot_name,
        output={'both': 'log'},
    )

    joint_state_publisher = Node(
        condition=IfCondition(use_joint_pub),
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}],
        namespace=robot_name,
        output={'both': 'log'},
    )

    joint_state_publisher_gui = Node(
        condition=IfCondition(use_joint_pub_gui),
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        namespace=robot_name,
        output={'both': 'log'},
    )

    rviz2 = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        namespace=robot_name,
        arguments=[
            '-d', rviz_config,
        ],
        parameters=[{
            'use_sim_time': use_sim_time,
        }],
        output={'both': 'log'},
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(robot_state_publisher)
    ld.add_action(joint_state_publisher)
    ld.add_action(joint_state_publisher_gui)
    ld.add_action(rviz2)
    return ld
