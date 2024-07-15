# Copyright (C) 2024 Intel Corporation
# Copyright 2023 Clearpath Robotics, Inc.
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

from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, Shutdown
from launch.actions import SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.conditions import UnlessCondition


def generate_launch_description():

    # Directories
    pkg_turtlebot4_ignition_bringup = get_package_share_directory(
        'turtlebot4_ignition_bringup')
    pkg_tb4_sim_scenario = get_package_share_directory(
        'tb4_sim_scenario')
    pkg_turtlebot4_ignition_gui_plugins = get_package_share_directory(
        'turtlebot4_ignition_gui_plugins')
    pkg_turtlebot4_description = get_package_share_directory(
        'turtlebot4_description')
    pkg_irobot_create_description = get_package_share_directory(
        'irobot_create_description')
    pkg_irobot_create_ignition_bringup = get_package_share_directory(
        'irobot_create_ignition_bringup')
    pkg_irobot_create_ignition_plugins = get_package_share_directory(
        'irobot_create_ignition_plugins')

    headless = LaunchConfiguration('headless')
    
    arguments = [
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              choices=['true', 'false'],
                              description='use_sim_time'),

        DeclareLaunchArgument('world', default_value=os.path.join(pkg_tb4_sim_scenario, 'worlds', 'maze.sdf'),
                              description='Simulation World File'),

        DeclareLaunchArgument('headless', default_value='False',
                              description='Whether to execute simulation gui'),
    ]

    # Set ignition resource path
    ign_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=[
            os.path.join(pkg_turtlebot4_ignition_bringup, 'worlds'), ':' +
            os.path.join(pkg_irobot_create_ignition_bringup, 'worlds'), ':' +
            str(Path(pkg_turtlebot4_description).parent.resolve()), ':' +
            str(Path(pkg_irobot_create_description).parent.resolve())]
    )

    ign_gui_plugin_path = SetEnvironmentVariable(
        name='IGN_GUI_PLUGIN_PATH',
        value=[
            os.path.join(pkg_turtlebot4_ignition_gui_plugins, 'lib'), ':' +
            os.path.join(pkg_irobot_create_ignition_plugins, 'lib')])

    env = {'GZ_SIM_SYSTEM_PLUGIN_PATH':
           ':'.join([os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', default=''),
                     os.environ.get('LD_LIBRARY_PATH', default='')]),
           'IGN_GAZEBO_SYSTEM_PLUGIN_PATH':  # TODO(CH3): To support pre-garden. Deprecated.
           ':'.join([os.environ.get('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', default=''),
                     os.environ.get('LD_LIBRARY_PATH', default='')])}
    # Ignition gazebo
    ignition_gazebo = ExecuteProcess(
        cmd=['ruby', '/usr/bin/ign', 'gazebo', LaunchConfiguration('world'), '-v', '4', '-r', '-s', '--force-version', '6'],
        output='screen',
        additional_env=env,
        on_exit=Shutdown(),
        sigterm_timeout='5',
        sigkill_timeout='10',
        log_cmd=True,
        emulate_tty=True
    )

    ignition_gazebo_gui = ExecuteProcess(
        condition=UnlessCondition(headless),
        cmd=['ruby', '/usr/bin/ign', 'gazebo', '-g', '-v', '4', '--gui-config',
             PathJoinSubstitution([pkg_turtlebot4_ignition_bringup, 'gui', 'gui.config']), '--force-version', '6'],
        output='screen',
        additional_env=env,
        on_exit=Shutdown(),
        sigterm_timeout='5',
        sigkill_timeout='10',
        log_cmd=True,
        emulate_tty=True
    )

    # Clock bridge
    clock_bridge = Node(package='ros_gz_bridge', executable='parameter_bridge',
                        name='clock_bridge',
                        output='screen',
                        arguments=[
                            '/clock' + '@rosgraph_msgs/msg/Clock' + '[ignition.msgs.Clock'
                        ])

    # Create launch description and add actions
    ld = LaunchDescription(arguments)
    ld.add_action(ign_resource_path)
    ld.add_action(ign_gui_plugin_path)
    ld.add_action(ignition_gazebo)
    ld.add_action(ignition_gazebo_gui)
    ld.add_action(clock_bridge)
    return ld
