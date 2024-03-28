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
from std_msgs.msg import Bool

from scenario_execution import ROSScenarioExecution
from scenario_execution.model.osc2_parser import OpenScenario2Parser
from scenario_execution.utils.logging import Logger


class TestRosTopicPublish(unittest.TestCase):
    # pylint: disable=missing-function-docstring,missing-class-docstring

    def setUp(self) -> None:
        rclpy.init()
        self.parser = OpenScenario2Parser(Logger('test', False))
        self.scenario_execution = ROSScenarioExecution()

        self.scenario_dir = get_package_share_directory('scenario_execution')

        self.received_msgs = []
        self.node = rclpy.create_node('test_node')
        self.srv = self.node.create_subscription(Bool, "/bla", self.callback, 10)

        self.executor = rclpy.executors.MultiThreadedExecutor()
        self.executor.add_node(self.node)
        self.executor_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.executor_thread.start()

    def tearDown(self):
        self.node.destroy_node()
        rclpy.try_shutdown()

    def callback(self, msg):
        self.received_msgs.append(msg)

    def test_success(self):
        scenarios = self.parser.process_file(os.path.join(
            self.scenario_dir, 'scenarios', 'test', 'test_ros_topic_publish.osc'), False)
        self.scenario_execution.scenarios = scenarios
        self.scenario_execution.run()
        self.assertTrue(self.scenario_execution.process_results())
        self.assertEqual(len(self.received_msgs), 1)
