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
import unittest
import threading

from ament_index_python.packages import get_package_share_directory

import rclpy
from std_msgs.msg import String

from scenario_execution_ros import ROSScenarioExecution
from scenario_execution.model.osc2_parser import OpenScenario2Parser
from scenario_execution.utils.logging import Logger


class TestRosCheckDataExternal(unittest.TestCase):
    # pylint: disable=missing-function-docstring,missing-class-docstring

    def setUp(self) -> None:
        rclpy.init()
        self.parser = OpenScenario2Parser(Logger('test', False))
        self.scenario_execution_ros = ROSScenarioExecution()

        self.scenario_dir = get_package_share_directory('scenario_execution_ros')

        self.received_msgs = []
        self.node = rclpy.create_node('test_node')
        self.publisher = self.node.create_publisher(String, "/foo", 10)
        self.srv = self.node.create_timer(1, self.timer_callback)

        self.executor = rclpy.executors.MultiThreadedExecutor()
        self.executor.add_node(self.node)
        self.executor_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.executor_thread.start()

    def timer_callback(self):
        self.publisher.publish(String(data="foo"))

    def tearDown(self):
        self.node.destroy_node()
        rclpy.try_shutdown()

    def callback(self, msg):
        self.received_msgs.append(msg)

    def test_success(self):
        tree = self.parser.process_file(os.path.join(
            self.scenario_dir, 'scenarios', 'test', 'test_ros_check_data_external.osc'), False)
        self.scenario_execution_ros.scenario_file = os.path.join(
            self.scenario_dir, 'scenarios', 'test', 'test_ros_check_data_external.osc')
        self.scenario_execution_ros.tree = tree
        self.scenario_execution_ros.run()
        self.assertTrue(self.scenario_execution_ros.process_results())
