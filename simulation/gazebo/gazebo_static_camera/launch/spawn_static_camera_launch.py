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
import tempfile
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.substitutions.path_join_substitution import PathJoinSubstitution

from launch_ros.actions import Node

ARGUMENTS = [
    DeclareLaunchArgument('camera_name', default_value='static_camera',
                          description='Camera name'),

    DeclareLaunchArgument('world_name', default_value='default',
                          description='World name'),

    DeclareLaunchArgument('update_rate', default_value='5',
                          description='Update rate of the sensor'),

    DeclareLaunchArgument('image_width', default_value='320',
                          description='Width of camera image'),

    DeclareLaunchArgument('image_height', default_value='240',
                          description='Height of camera image'),
]

for pose_element in ['x', 'y', 'z', 'roll', 'pitch', 'yaw']:
    ARGUMENTS.append(DeclareLaunchArgument(pose_element, default_value='0.0',
                     description=f'{pose_element} component of the camera pose.'))


def generate_launch_description():
    gazebo_static_camera_dir = get_package_share_directory('gazebo_static_camera')

    camera_name = LaunchConfiguration('camera_name')
    world_name = LaunchConfiguration('world_name')
    update_rate = LaunchConfiguration('update_rate')
    image_width = LaunchConfiguration('image_width')
    image_height = LaunchConfiguration('image_height')
    camera_sdf = tempfile.NamedTemporaryFile(prefix='gazebo_static_camera_', suffix='.sdf', delete=False)

    x, y, z = LaunchConfiguration('x'), LaunchConfiguration('y'), LaunchConfiguration('z')
    roll, pitch, yaw = LaunchConfiguration('roll'), LaunchConfiguration('pitch'), LaunchConfiguration('yaw')

    camera_sdf_xacro = ExecuteProcess(
        cmd=['xacro', '-o', camera_sdf.name, ['update_rate:=', update_rate], ['image_width:=', image_width], ['image_height:=', image_height], PathJoinSubstitution([gazebo_static_camera_dir, 'models', 'camera.sdf.xacro'])])

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
             '[ignition.msgs.Image'],
            ['/world/', world_name,
             '/model/', camera_name,
             '/link/link/sensor/camera/camera_info' +
             '@sensor_msgs/msg/CameraInfo' +
             '[ignition.msgs.CameraInfo'],
        ],
        remappings=[
            (['/world/', world_name, '/model/', camera_name, '/link/link/sensor/camera/image'],
             [camera_name, '/image_raw']),
            (['/world/', world_name, '/model/', camera_name, '/link/link/sensor/camera/camera_info'],
             [camera_name, '/camera_info'])
        ]
    )

    spawn_camera = Node(
        package='ros_ign_gazebo',
        executable='create',
        arguments=['-name', camera_name,
                   '-x', x,
                   '-y', y,
                   '-z', z,
                   '-R', roll,
                   '-P', pitch,
                   '-Y', yaw,
                   '-file', camera_sdf.name],
        output='screen'
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(camera_sdf_xacro)
    ld.add_action(camera_bridge)
    ld.add_action(spawn_camera)
    return ld
