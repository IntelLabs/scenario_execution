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
from setuptools import find_namespace_packages, setup

PACKAGE_NAME = 'scenario_execution_ros'

setup(
    name=PACKAGE_NAME,
    version='1.2.0',
    packages=find_namespace_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + PACKAGE_NAME]),
        ('share/' + PACKAGE_NAME, ['package.xml']),
        (os.path.join('share', PACKAGE_NAME, 'scenarios'), glob('scenarios/*.osc')),
        (os.path.join('share', PACKAGE_NAME, 'scenarios', 'test'), glob('scenarios/test/*osc')),
        (os.path.join('share', PACKAGE_NAME, 'scenarios', 'test'), glob('scenarios/test/*py')),
        (os.path.join('share', PACKAGE_NAME, 'launch'), glob('launch/*launch.py'))
    ],
    install_requires=[
        'setuptools',
        'transforms3d==0.3.1',
    ],
    zip_safe=True,
    maintainer='Intel Labs',
    maintainer_email='scenario-execution@intel.com',
    description='Scenario Execution for ROS',
    license='Apache License 2.0',
    tests_require=['pytest'],
    include_package_data=True,
    entry_points={
        'console_scripts': [
            'scenario_execution_ros = scenario_execution_ros.scenario_execution_ros:main',
        ],
        'scenario_execution.actions': [
            'action_call = scenario_execution_ros.actions.ros_action_call:RosActionCall',
            'assert_topic_latency = scenario_execution_ros.actions.assert_topic_latency:AssertTopicLatency',
            'assert_tf_moving = scenario_execution_ros.actions.assert_tf_moving:AssertTfMoving',
            'assert_lifecycle_state = scenario_execution_ros.actions.assert_lifecycle_state:AssertLifecycleState',
            'bag_play = scenario_execution_ros.actions.ros_bag_play:RosBagPlay',
            'bag_record = scenario_execution_ros.actions.ros_bag_record:RosBagRecord',
            'check_data = scenario_execution_ros.actions.ros_topic_check_data:RosTopicCheckData',
            'check_data_external = scenario_execution_ros.actions.ros_topic_check_data_external:RosTopicCheckDataExternal',
            'differential_drive_robot.odometry_distance_traveled = scenario_execution_ros.actions.odometry_distance_traveled:OdometryDistanceTraveled',
            'differential_drive_robot.tf_close_to = scenario_execution_ros.actions.tf_close_to:TfCloseTo',
            'log_check = scenario_execution_ros.actions.ros_log_check:RosLogCheck',
            'ros_launch = scenario_execution_ros.actions.ros_launch:RosLaunch',
            'service_call = scenario_execution_ros.actions.ros_service_call:RosServiceCall',
            'set_node_parameter = scenario_execution_ros.actions.ros_set_node_parameter:RosSetNodeParameter',
            'topic_monitor = scenario_execution_ros.actions.ros_topic_monitor:RosTopicMonitor',
            'topic_publish = scenario_execution_ros.actions.ros_topic_publish:RosTopicPublish',
            'wait_for_data = scenario_execution_ros.actions.ros_topic_wait_for_data:RosTopicWaitForData',
            'wait_for_nodes = scenario_execution_ros.actions.ros_wait_for_nodes:RosWaitForNodes',
            'wait_for_topics = scenario_execution_ros.actions.ros_topic_wait_for_topics:RosTopicWaitForTopics',
        ],
        'scenario_execution.osc_libraries': [
            'ros = '
            'scenario_execution_ros.get_osc_library:get_ros_library',
        ]
    },
)
