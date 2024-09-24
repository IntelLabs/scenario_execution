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

from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

ARGUMENTS = [
    DeclareLaunchArgument('ros2_control_hardware_type', default_value='mock_components',
                          choices=['ignition', 'mock_components'],
                          description='ROS2 control hardware interface type to use for the launch file'),
    DeclareLaunchArgument('use_sim_time', default_value='true',
                          choices=['true', 'false'],
                          description='use_sim_time'),
]


def generate_launch_description():
    pkg_arm_sim_scenario = get_package_share_directory('arm_sim_scenario')
    ros2_control_hardware_type = LaunchConfiguration('ros2_control_hardware_type')
    use_sim_time = LaunchConfiguration('use_sim_time')

    moveit_config_builder = MoveItConfigsBuilder("moveit_resources_panda")

    moveit_config_builder._MoveItConfigsBuilder__urdf_package = pkg_arm_sim_scenario  # pylint: disable=W0212
    moveit_config_builder._MoveItConfigsBuilder__urdf_file_path = Path("urdf/arm.urdf.xacro")  # pylint: disable=W0212

    moveit_config = (
        moveit_config_builder
        .robot_description(
            mappings={
                "ros2_control_hardware_type": ros2_control_hardware_type
            },
        )
        .robot_description_semantic(file_path="config/panda.srdf")
        .trajectory_execution(file_path="config/gripper_moveit_controllers.yaml")
        .planning_pipelines(
            pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"]
        )
        # .planning_scene_monitor(publish_robot_description=True)
        .to_moveit_configs()
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[{'use_sim_time': use_sim_time, },
                    moveit_config.to_dict()],
        arguments=["--ros-args", "--log-level", "info"],
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(move_group_node)
    return ld
