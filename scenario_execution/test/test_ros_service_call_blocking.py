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
import time
import unittest
import rclpy
import threading
from scenario_execution import ROSScenarioExecution
from scenario_execution_base.model.osc2_parser import OpenScenario2Parser
from scenario_execution_base.utils.logging import Logger
from ament_index_python.packages import get_package_share_directory

from std_srvs.srv import SetBool
from std_msgs.msg import Int32

os.environ["PYTHONUNBUFFERED"] = '1'


class TestScenarioExectionSuccess(unittest.TestCase):
    # pylint: disable=missing-function-docstring

    def setUp(self):
        rclpy.init()
        self.received_msgs = []
        self.node = rclpy.create_node('test_node')

        self.executor = rclpy.executors.MultiThreadedExecutor()
        self.executor.add_node(self.node)
        self.executor_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.executor_thread.start()

        self.callback_group = rclpy.callback_groups.ReentrantCallbackGroup()
        self.srv = self.node.create_service(
            SetBool, "/bla_service", self.service_callback, callback_group=self.callback_group)
        self.sub = self.node.create_subscription(
            Int32, "/bla", self.topic_callback, 10, callback_group=self.callback_group)

        self.scenario_dir = get_package_share_directory('scenario_execution')
        self.parser = OpenScenario2Parser(Logger('test', False))
        self.scenario_execution = ROSScenarioExecution()

    def tearDown(self):
        self.node.destroy_node()
        rclpy.try_shutdown()

    def service_callback(self, msg, response):
        self.assertTrue(msg.data, "Invalid request received")
        current_time = self.node.get_clock().now()
        end_time = current_time + rclpy.duration.Duration(seconds=5)
        while current_time <= end_time:
            time.sleep(0.1)
            current_time = self.node.get_clock().now()
        return response

    def topic_callback(self, msg):
        current_time = self.node.get_clock().now()
        # print(f"Received {msg}")
        # if self.received_msgs:
        #     print(f"Since last: {current_time - self.received_msgs[-1][0]}")
        self.received_msgs.append((current_time, msg))

    def test_success(self):
        scenarios = self.parser.process_file(os.path.join(
            self.scenario_dir, 'scenarios', 'test', 'test_ros_service_call_blocking.osc'), False)
        self.scenario_execution.scenarios = scenarios
        self.scenario_execution.run()
        self.assertTrue(self.scenario_execution.process_results())

        self.assertGreater(len(self.received_msgs), 0)
        prev_elem = self.received_msgs[0]
        for elem in self.received_msgs[1:]:
            self.assertGreater(elem[1].data, prev_elem[1].data)
            time_since_last = elem[0]-prev_elem[0]
            self.assertGreaterEqual(time_since_last, rclpy.duration.Duration(seconds=0.5))
            self.assertLessEqual(time_since_last, rclpy.duration.Duration(seconds=1.9))
            prev_elem = elem
