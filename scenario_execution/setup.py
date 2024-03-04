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

""" Setup python package """
from glob import glob
import os
from setuptools import find_packages, setup

PACKAGE_NAME = 'scenario_execution'

setup(
    name=PACKAGE_NAME,
    version='1.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + PACKAGE_NAME]),
        ('share/' + PACKAGE_NAME, ['package.xml']),
        (os.path.join('share', PACKAGE_NAME, 'scenarios'), glob('scenarios/*.osc')),
        (os.path.join('share', PACKAGE_NAME, 'scenarios', 'test'), glob('scenarios/test/*osc')),
        (os.path.join('share', PACKAGE_NAME, 'launch'), glob('launch/*launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Intel Labs',
    maintainer_email='scenario-execution@intel.com',
    description='Scenario Execution for ROS',
    license='Apache License 2.0',
    tests_require=['pytest'],
    include_package_data=True,
    entry_points={
        'console_scripts': [
            'scenario_execution = scenario_execution.scenario_execution_ros:main',
        ],
        'scenario_execution.actions': [
            'differential_drive_robot.init_nav2 = scenario_execution.actions.init_nav2:InitNav2',
            'differential_drive_robot.nav_to_pose = scenario_execution.actions.nav_to_pose:NavToPose',
            'differential_drive_robot.nav_through_poses = scenario_execution.actions.nav_through_poses:NavThroughPoses',
            'wait_for_data = scenario_execution.actions.ros_topic_wait_for_data:RosTopicWaitForData',
            'check_data = scenario_execution.actions.ros_topic_check_data:RosTopicCheckData',
            'service_call = scenario_execution.actions.ros_service_call:RosServiceCall',
            'topic_publish = scenario_execution.actions.ros_topic_publish:RosTopicPublish',
            'odometry_distance_traveled = '
            'scenario_execution.actions.odometry_distance_traveled:OdometryDistanceTraveled',
            'set_node_parameter = scenario_execution.actions.ros_set_node_parameter:RosSetNodeParameter',
            'record_bag = scenario_execution.actions.ros_bag_record:RosBagRecord',
            'wait_for_topics = scenario_execution.actions.ros_topic_wait_for_topics:RosTopicWaitForTopics',
            'log_check = scenario_execution.actions.ros_log_check:RosLogCheck',
            'differential_drive_robot.tf_close_to = scenario_execution.actions.tf_close_to:TfCloseTo',
        ],
        'scenario_execution.osc_libraries': [
            'ros = '
            'scenario_execution.get_osc_library:get_ros_library',
        ]
    },
)
