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
import rclpy
import threading
from scenario_execution import ROSScenarioExecution
from scenario_execution_base.model.osc2_parser import OpenScenario2Parser
from scenario_execution_base.utils.logging import Logger
from ament_index_python.packages import get_package_share_directory

from std_msgs.msg import Empty

os.environ["PYTHONUNBUFFERED"] = '1'


class Test(unittest.TestCase):
    # pylint: disable=missing-function-docstring,missing-class-docstring
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self) -> None:
        self.parser = OpenScenario2Parser(Logger('test'))
        self.scenario_execution = ROSScenarioExecution()

        self.scenario_dir = get_package_share_directory('scenario_execution')

        self.node = rclpy.create_node('test_node')
        self.pub = self.node.create_publisher(Empty, "/bla", 10)
        self.timer = self.node.create_timer(1, self.timer_callback)

        self.executor = rclpy.executors.MultiThreadedExecutor()
        self.executor.add_node(self.node)
        self.executor_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.executor_thread.start()

    def tearDown(self):
        self.node.destroy_node()

    def timer_callback(self):
        self.pub.publish(Empty())

    def test_success(self):
        scenarios = self.parser.process_file(os.path.join(
            self.scenario_dir, 'scenarios', 'test', 'test_ros_event_to_blackboard.osc'), False)
        self.assertIsNotNone(scenarios)
        self.scenario_execution.scenarios = scenarios
        ret = self.scenario_execution.run()
        self.assertTrue(ret)