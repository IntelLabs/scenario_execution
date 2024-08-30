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
from launch.actions import (
    DeclareLaunchArgument,
)
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    Command
)
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
import yaml


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None


ARGUMENTS = [
    DeclareLaunchArgument('robot_model', default_value='wx200',
                          choices=['wx200'],
                          description='robot_model type of the Interbotix Arm'),
    DeclareLaunchArgument('robot_name', default_value=LaunchConfiguration('robot_model'),
                          description='Robot name'),
    DeclareLaunchArgument('use_rviz_moveit', default_value='false',
                          choices=['true', 'false'],
                          description='launches RViz if set to `true`.'),
    DeclareLaunchArgument('use_sim_time', default_value='true',
                          choices=['true', 'false'],
                          description='use_sim_time'),
    DeclareLaunchArgument('rviz_config',
                          default_value=PathJoinSubstitution([get_package_share_directory('arm_sim_scenario'),
                                                              'rviz',
                                                              'xsarm_moveit.rviz',
                                                              ]),
                          description='file path to the config file RViz should load.',),
    DeclareLaunchArgument('hardware_type',
                          default_value='fake',
                          choices=['actual', 'fake', 'gz_classic', 'ignition'],
                          description='configure robot_description to use actual, fake, or simulated hardware',
                          ),
    DeclareLaunchArgument('rviz_frame', default_value='world',
                          description=(
                              'defines the fixed frame parameter in RViz. Note that if `use_world_frame` is '
                              '`false`, this parameter should be changed to a frame that exists.'
                          ),
                          )
]


def generate_launch_description():

    pkg_arm_sim_scenario = get_package_share_directory('arm_sim_scenario')

    robot_model = LaunchConfiguration('robot_model')
    robot_name = LaunchConfiguration('robot_name')
    use_rviz_moveit = LaunchConfiguration('use_rviz_moveit')
    rviz_config = LaunchConfiguration('rviz_config')
    rviz_frame = LaunchConfiguration('rviz_frame')
    use_sim_time = LaunchConfiguration('use_sim_time')
    hardware_type = LaunchConfiguration('hardware_type')

    xacro_file = PathJoinSubstitution([pkg_arm_sim_scenario,
                                       'urdf',
                                       'wx200.urdf.xacro'])
    srdf_xacro_file = PathJoinSubstitution([pkg_arm_sim_scenario,
                                            'config', 'srdf',
                                            'wx200.srdf.xacro'])

    kinematics_config = PathJoinSubstitution([
        pkg_arm_sim_scenario,
        'config',
        'kinematics.yaml',
    ])

    robot_description = {'robot_description': ParameterValue(Command([
        'xacro', ' ',xacro_file,' ',
        'gazebo:=ignition',' ',
        'base_link_frame:=','base_link ',
        'use_world_frame:=','true ',
        'robot_model:=',robot_model,' ',
        'robot_name:=',robot_name,' ',
        'hardware_type:=',hardware_type]), value_type=str)}

    robot_description_semantic = {'robot_description_semantic': ParameterValue(Command([
        'xacro', ' ', srdf_xacro_file,' ',
        'robot_name:=', robot_name,' ',
        'base_link_frame:=', 'base_link ',
        'use_world_frame:=','true ',
        'hardware_type:=', hardware_type]), value_type=str)}

    ompl_planning_pipeline_config = {
        'move_group': {
            'planning_plugin':
                'ompl_interface/OMPLPlanner',
            'request_adapters':
                'default_planner_request_adapters/AddTimeOptimalParameterization '
                'default_planner_request_adapters/FixWorkspaceBounds '
                'default_planner_request_adapters/FixStartStateBounds '
                'default_planner_request_adapters/FixStartStateCollision '
                'default_planner_request_adapters/FixStartStatePathConstraints',
            'start_state_max_bounds_error':
                0.1,
        }
    }

    ompl_planning_pipeline_yaml_file = load_yaml(
        'arm_sim_scenario', 'config/ompl_planning.yaml'
    )

    ompl_planning_pipeline_config['move_group'].update(ompl_planning_pipeline_yaml_file)

    controllers_config = load_yaml(
        'arm_sim_scenario',
        'config/controllers/wx200_controllers.yaml'
    )

    config_joint_limits = load_yaml(
        'arm_sim_scenario',
        f'config/joint_limits/wx200_joint_limits.yaml'
    )

    joint_limits = {
        'robot_description_planning': config_joint_limits,
    }

    moveit_controllers = {
        'moveit_simple_controller_manager':
            controllers_config,
        'moveit_controller_manager':
            'moveit_simple_controller_manager/MoveItSimpleControllerManager',
    }

    trajectory_execution_parameters = {
        'moveit_manage_controllers': True,
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.01,
    }

    planning_scene_monitor_parameters = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
    }

    sensor_parameters = {
        'sensors': [''],
    }

    remappings = [
        (
            [robot_name, '/get_planning_scene'],
            [robot_name, '/get_planning_scene']
        ),
        (
            '/arm_controller/follow_joint_trajectory', [robot_name, '/arm_controller/follow_joint_trajectory']

        ),
        (
            '/gripper_controller/follow_joint_trajectory', [robot_name, '/gripper_controller/follow_joint_trajectory']

        ),
    ]

    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        parameters=[
            {
                'planning_scene_monitor_options': {
                    'robot_description': 'robot_description',
                    'joint_state_topic': [robot_name, '/joint_states'],
                },
                'use_sim_time': use_sim_time,
            },
            robot_description,
            robot_description_semantic,
            kinematics_config,
            ompl_planning_pipeline_config,
            trajectory_execution_parameters,
            moveit_controllers,
            planning_scene_monitor_parameters,
            joint_limits,
            sensor_parameters,
        ],
        remappings=remappings,
        output={'both': 'screen'},
    )

    moveit_rviz_node = Node(
        condition=IfCondition(use_rviz_moveit),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=[
            '-d', rviz_config,
            '-f', rviz_frame,
        ],
        parameters=[
            robot_description,
            robot_description_semantic,
            ompl_planning_pipeline_config,
            kinematics_config,
            {'use_sim_time': use_sim_time},
        ],
        remappings=remappings,
        output={'both': 'log'},
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(move_group_node)
    ld.add_action(moveit_rviz_node)
    return ld
