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
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


ARGUMENTS = [
    DeclareLaunchArgument('robot_model', default_value='wx200',
                          choices=['wx200'],
                          description='model type of the Interbotix Arm'),
    DeclareLaunchArgument('robot_name', default_value='wx200',
                          description='Robot name'),
    DeclareLaunchArgument('use_rviz', default_value='false',
                          choices=['true', 'false'],
                          description='launches RViz if set to `true`.'),
    DeclareLaunchArgument('use_sim_time', default_value='true',
                          choices=['true', 'false'],
                          description='use_sim_time'),
    DeclareLaunchArgument('headless', default_value='False',
                          description='Whether to execute simulation gui'),
    DeclareLaunchArgument('hardware_type',
                          default_value='fake',
                          choices=['actual', 'fake', 'gz_classic', 'ignition'],
                          description='configure robot_description to use actual, fake, or simulated hardware',
                          ),
    DeclareLaunchArgument('use_joint_pub_gui',
                          default_value='false',
                          choices=('true', 'false'),
                          description='launches the joint_state_publisher GUI.',
                          ),
    DeclareLaunchArgument('waist_initial_value',
                          default_value='0',
                          description='waist joint initial value.',
                          ),
    DeclareLaunchArgument('shoulder_initial_value',
                          default_value='0',
                          description='shoulder joint initial value.',
                          ),
    DeclareLaunchArgument('elbow_initial_value',
                          default_value='0',
                          description='elbow joint initial value.',
                          ),
    DeclareLaunchArgument('wrist_angle_initial_value',
                          default_value='0',
                          description='wrist_angle joint initial value.',
                          ),
    DeclareLaunchArgument('wrist_rotate_initial_value',
                          default_value='0',
                          description='wrist_rotate joint initial value.',
                          ),
    DeclareLaunchArgument('finger_initial_value',
                          default_value='0',
                          description='gripper finger initial value.',
                          ),

]


def generate_launch_description():
    robot_model = LaunchConfiguration('robot_model')
    robot_name = LaunchConfiguration('robot_name')
    use_sim_time = LaunchConfiguration('use_sim_time')
    hardware_type = LaunchConfiguration('hardware_type')
    # Intial joint states
    waist_initial_value = LaunchConfiguration('waist_initial_value')
    shoulder_initial_value = LaunchConfiguration('shoulder_initial_value')
    elbow_initial_value = LaunchConfiguration('elbow_initial_value')
    wrist_angle_initial_value = LaunchConfiguration('wrist_angle_initial_value')
    wrist_rotate_initial_value = LaunchConfiguration('wrist_rotate_initial_value')
    finger_initial_value = LaunchConfiguration('finger_initial_value')

    pkg_arm_sim_scenario = get_package_share_directory('arm_sim_scenario')

    ros2_control_controllers_config_parameter_file = ParameterFile(
        param_file=PathJoinSubstitution([
            FindPackageShare('arm_sim_scenario'),
            'config',
            'trajectory_controllers',
            'wx200_trajectory_controllers.yaml',
        ]),
        allow_substs=True
    )

    xacro_file = PathJoinSubstitution([pkg_arm_sim_scenario,
                                       'urdf',
                                       'wx200.urdf.xacro'])
    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        namespace=robot_name,
        parameters=[
            {'robot_description': ParameterValue(Command([
                'xacro ', xacro_file, ' ',
                'robot_model:=', robot_model, ' ',
                'robot_name:=', robot_name, ' ',
                'use_world_frame:=', 'true ',
                'waist_initial_value:=', waist_initial_value, ' ',
                'shoulder_initial_value:=', shoulder_initial_value, ' ',
                'elbow_initial_value:=', elbow_initial_value, ' ',
                'wrist_angle_initial_value:=', wrist_angle_initial_value, ' ',
                'wrist_rotate_initial_value:=', wrist_rotate_initial_value, ' ',
                'finger_initial_value:=', finger_initial_value, ' ',
                'hardware_type:=', hardware_type]), value_type=str)},
            ros2_control_controllers_config_parameter_file,
        ],
        output={'both': 'screen'},
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

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(controller_manager_node)
    ld.add_action(spawn_joint_state_broadcaster_node)
    ld.add_action(spawn_arm_controller_node)
    ld.add_action(spawn_gripper_controller_node)
    return ld
