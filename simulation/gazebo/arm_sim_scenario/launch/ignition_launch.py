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

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler, IncludeLaunchDescription, Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


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
                          default_value='ignition',
                          choices=['actual', 'fake', 'gz_classic', 'ignition'],
                          description='configure robot_description to use actual, fake, or simulated hardware',
                          ),
    DeclareLaunchArgument('use_joint_pub_gui',
                          default_value='false',
                          choices=('true', 'false'),
                          description='launches the joint_state_publisher GUI.',
                          ),
    DeclareLaunchArgument('world', default_value='maze',
                          description='Ignition World'),

]


def generate_launch_description():
    pkg_arm_sim_scenario = get_package_share_directory(
        'arm_sim_scenario')
    robot_model = LaunchConfiguration('robot_model')
    robot_name = LaunchConfiguration('robot_name')
    use_rviz = LaunchConfiguration('use_rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')
    hardware_type = LaunchConfiguration('hardware_type')
    use_joint_pub_gui = LaunchConfiguration('use_joint_pub_gui')

    env = {'IGN_GAZEBO_SYSTEM_PLUGIN_PATH': os.environ['LD_LIBRARY_PATH'],
           'IGN_GAZEBO_RESOURCE_PATH': os.path.dirname(
        get_package_share_directory('arm_sim_scenario'))}

    ignition_gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo', [PathJoinSubstitution([pkg_arm_sim_scenario, 'worlds', LaunchConfiguration('world')]), '.sdf'], '-r', '-v', '4'],
        output='screen',
        additional_env=env,
        on_exit=Shutdown(),
        sigterm_timeout='5',
        sigkill_timeout='10',
        log_cmd=True,
        emulate_tty=True
    )

    spawn_robot_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'spawn_wx200',
                   '-x', '0.0',
                   '-y', '0.0',
                   '-z', '0.0',
                   '-Y', '0.0',
                   '-topic', 'wx200/robot_description'],
        output='screen'
    )

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'],
        output='screen'
    )

    arm_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('arm_sim_scenario'),
                'launch',
                'arm_description_launch.py'
            ])
        ]),
        launch_arguments={
            'robot_model': robot_model,
            'robot_name': robot_name,
            'use_rviz': use_rviz,
            'use_sim_time': use_sim_time,
            'hardware_type': hardware_type,
            'use_joint_pub_gui': use_joint_pub_gui
        }.items(),
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

    load_joint_state_broadcaster_event = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_robot_node,
            on_exit=[spawn_joint_state_broadcaster_node]
        )
    )

    load_arm_controller_event = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_joint_state_broadcaster_node,
            on_exit=[spawn_arm_controller_node]
        )
    )

    load_gripper_controller_event = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_joint_state_broadcaster_node,
            on_exit=[spawn_gripper_controller_node]
        )
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(ignition_gazebo)
    ld.add_action(spawn_robot_node)
    ld.add_action(load_joint_state_broadcaster_event)
    ld.add_action(load_arm_controller_event)
    ld.add_action(load_gripper_controller_event)
    ld.add_action(arm_description_launch)
    ld.add_action(clock_bridge)
    return ld
