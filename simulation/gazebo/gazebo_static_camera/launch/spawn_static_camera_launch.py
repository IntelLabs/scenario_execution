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
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression, EnvironmentVariable
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from launch.conditions import IfCondition, UnlessCondition

from launch_ros.actions import Node

ARGUMENTS = [
    DeclareLaunchArgument('camera_name', default_value='static_camera',
                          description='Camera name'),

    DeclareLaunchArgument('world_name', default_value='default',
                          description='World name'),
]

for pose_element in ['x', 'y', 'z', 'roll', 'pitch', 'yaw']:
    ARGUMENTS.append(DeclareLaunchArgument(pose_element, default_value='0.0',
                     description=f'{pose_element} component of the camera pose.'))


def generate_launch_description():
    gazebo_static_camera_dir = get_package_share_directory('gazebo_static_camera')

    camera_name = LaunchConfiguration('camera_name')
    world_name = LaunchConfiguration('world_name')

    x, y, z = LaunchConfiguration('x'), LaunchConfiguration('y'), LaunchConfiguration('z')
    roll, pitch, yaw = LaunchConfiguration('roll'), LaunchConfiguration('pitch'), LaunchConfiguration('yaw')

    if os.environ['ROS_DISTRO'] == 'humble':
        sim_prefix = 'ignition'
    else:
        sim_prefix = 'gz'

    camera_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=[
            ['/world/', world_name,
             '/model/', camera_name,
             '/link/link/sensor/camera/image' +
             '@sensor_msgs/msg/Image' +
             '[' + sim_prefix + '.msgs.Image'],
            ['/world/', world_name,
             '/model/', camera_name,
             '/link/link/sensor/camera/camera_info' +
             '@sensor_msgs/msg/CameraInfo' +
             '[' + sim_prefix + '.msgs.CameraInfo'],
        ],
        remappings=[
            (['/world/', world_name, '/model/', camera_name, '/link/link/sensor/camera/image'],
             [camera_name, '/image_raw']),
            (['/world/', world_name, '/model/', camera_name, '/link/link/sensor/camera/camera_info'],
             [camera_name, '/camera_info'])
        ]
    )

    is_humble = PythonExpression(["'", EnvironmentVariable('ROS_DISTRO'), "'== 'humble'"])

    spawn_camera_humble = Node(
        condition=IfCondition(is_humble),
        package='ros_ign_gazebo',
        executable='create',
        arguments=['-name', camera_name,
                   '-x', x,
                   '-y', y,
                   '-z', z,
                   '-R', roll,
                   '-P', pitch,
                   '-Y', yaw,
                   '-file', PathJoinSubstitution([gazebo_static_camera_dir, 'models', 'camera.sdf'])],
        output='screen'
    )
    spawn_camera = Node(
        condition=UnlessCondition(is_humble),
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', camera_name,
                   '-x', x,
                   '-y', y,
                   '-z', z,
                   '-R', roll,
                   '-P', pitch,
                   '-Y', yaw,
                   '-file', PathJoinSubstitution([gazebo_static_camera_dir, 'models', 'camera.sdf'])],
        output='screen'
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(camera_bridge)
    ld.add_action(spawn_camera_humble)
    ld.add_action(spawn_camera)
    return ld
