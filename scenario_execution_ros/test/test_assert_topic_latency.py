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

import unittest
import threading

from ament_index_python.packages import get_package_share_directory

import rclpy
from std_msgs.msg import String

from scenario_execution_ros import ROSScenarioExecution
from scenario_execution.model.osc2_parser import OpenScenario2Parser
from scenario_execution.model.model_to_py_tree import create_py_tree
from scenario_execution.utils.logging import Logger
from antlr4.InputStream import InputStream


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
        self.publishing_thread = threading.Thread(target=self.publish_messages, daemon=True) # TODO: use a timer 
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
        scenario_content = """
import osc.ros

scenario test_assert_topic_latency:
    do parallel:
        serial:
            assert_topic_latency(
                topic_name: '/bla',
                latency: 0.001s
            emit end
"""
        parsed_tree = self.parser.parse_input_stream(InputStream(scenario_content))
        model = self.parser.create_internal_model(parsed_tree, "test.osc", False)
        scenarios = create_py_tree(model, self.parser.logger, False)
        self.scenario_execution_ros.scenarios = scenarios
        self.scenario_execution_ros.run()
        self.assertTrue(self.scenario_execution_ros.process_results())

    def test_failure(self):
        scenario_content = """
import osc.ros

scenario test_assert_topic_latency:
    do parallel:
        serial:
            assert_topic_latency(
                topic_name: '/bla',
                latency: 0.001s,
                fail_on_finish: true)
            emit end
"""
        parsed_tree = self.parser.parse_input_stream(InputStream(scenario_content))
        model = self.parser.create_internal_model(parsed_tree, "test.osc", False)
        scenarios = create_py_tree(model, self.parser.logger, False)
        self.scenario_execution_ros.scenarios = scenarios
        self.scenario_execution_ros.run()
        self.assertFalse(self.scenario_execution_ros.process_results())

    def test_running(self):
        scenario_content = """
import osc.ros

scenario test_assert_topic_latency:
    do parallel:
        serial:
            assert_topic_latency(
                topic_name: '/bla',
                latency: 1.5s)
            emit fail
        serial:
            wait elapsed(10s)
            emit end
"""
        parsed_tree = self.parser.parse_input_stream(InputStream(scenario_content))
        model = self.parser.create_internal_model(parsed_tree, "test.osc", False)
        scenarios = create_py_tree(model, self.parser.logger, False)
        self.scenario_execution_ros.scenarios = scenarios
        self.scenario_execution_ros.run()
        self.assertTrue(self.scenario_execution_ros.process_results())


# TODO:
# - test different comparison operators
# - test rolling_average_count
# - test wait_for_first_message

