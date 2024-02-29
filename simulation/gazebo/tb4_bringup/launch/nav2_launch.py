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
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


ARGUMENTS = [

    DeclareLaunchArgument('namespace', default_value='',
                          description='Robot namespace'),
    # Ignition setup
    DeclareLaunchArgument('world', default_value='warehouse',
                          description='Ignition World'),

    DeclareLaunchArgument('model', default_value='standard',
                          choices=['standard', 'lite'],
                          description='Turtlebot4 Model'),
    DeclareLaunchArgument('launch_simulation', default_value='True',
                          description='Set "false" to skip simulation startup.'),
    DeclareLaunchArgument('headless', default_value='True',
                          description='Start Igniton GUI or not'),

    DeclareLaunchArgument('map_yaml', default_value=[LaunchConfiguration('world'), '.yaml'],
                          description='map yaml file'),
]

# Inital robot pose in the world
for pose_element in ['x', 'y', 'z', 'yaw']:
    ARGUMENTS.append(DeclareLaunchArgument(pose_element,
                                           default_value='0.0',
                                           description=f'{pose_element} component of the robot pose.'))


def generate_launch_description():

    # Directories
    pkg_tb4_bringup = get_package_share_directory('tb4_bringup')
    pkg_tb4_nav = get_package_share_directory('turtlebot4_navigation')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    # Launch args
    map_yaml = LaunchConfiguration('map_yaml')

    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution([pkg_nav2_bringup, 'launch', 'bringup_launch.py'])]),
        launch_arguments={
            'map': PathJoinSubstitution([pkg_tb4_nav, 'maps', map_yaml]),
            'use_sim_time': 'True',
            'use_composition': 'False',
            'params_file': PathJoinSubstitution([pkg_tb4_bringup, 'params', 'nav2_params.yaml']),
            'autostart': 'True'}.items()
    )

    # Create launch description
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(nav2_bringup)
    return ld
