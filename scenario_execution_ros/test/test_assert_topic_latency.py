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


class TestScenarioExectionSuccess(unittest.TestCase):
    # pylint: disable=missing-function-docstring

    def setUp(self) -> None:
        rclpy.init()
        self.running = True
        self.parser = OpenScenario2Parser(Logger('test', False))
        self.scenario_execution_ros = ROSScenarioExecution()
        self.scenario_dir = get_package_share_directory('scenario_execution_ros')
        self.received_msgs = []
        self.node = rclpy.create_node('test_node')
        self.publisher = self.node.create_publisher(String, "/bla", 10)
        self.publishing_thread = threading.Thread(target=self.publish_messages, daemon=True)
        self.publishing_thread.start()
        self.executor = rclpy.executors.MultiThreadedExecutor()
        self.executor.add_node(self.node)
        self.executor_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.executor_thread.start()

    def publish_messages(self):
        while self.running:
            msg = String()
            msg.data = 'Hello'
            self.publisher.publish(msg)

    def tearDown(self):
        self.running = False
        self.publishing_thread.join()
        self.node.destroy_node()
        rclpy.try_shutdown()

    def test_success(self):
        scenarios = self.parser.process_file(os.path.join(
            self.scenario_dir, 'scenarios', 'test', 'test_assert_topic_latency.osc'), False)
        self.scenario_execution_ros.scenarios = scenarios
        self.scenario_execution_ros.run()
        self.assertTrue(self.scenario_execution_ros.process_results())
