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
        self.publish_timer = self.node.create_timer(1, self.publish_messages)
        self.executor = rclpy.executors.MultiThreadedExecutor()
        self.executor.add_node(self.node)
        self.executor_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.executor_thread.start()

    def publish_messages(self):
        msg = String()
        msg.data = 'Hello'
        self.publisher.publish(msg)

    def tearDown(self):
        self.running = False
        self.node.destroy_node()
        rclpy.try_shutdown()


# test verifies that the action succeeds only when 'fail_on_finish' is set to false.
# By default, 'fail_on_finish' is true and the action fails if the actual latency exceeds the threshold.


    def test_success(self):
        scenario_content = """
import osc.ros

scenario test_assert_topic_latency:
    do parallel:
        serial:
            assert_topic_latency(
                topic_name: '/bla',
                latency: 0.5s,
                fail_on_finish: false)
            emit end
"""
        parsed_tree = self.parser.parse_input_stream(InputStream(scenario_content))
        model = self.parser.create_internal_model(parsed_tree, "test.osc", False)
        scenarios = create_py_tree(model, self.parser.logger, False)
        self.scenario_execution_ros.scenarios = scenarios
        self.scenario_execution_ros.run()
        self.assertTrue(self.scenario_execution_ros.process_results())


# verifies the successful execution of the action when the correct 'topic_type' is set.


    def test_success_with_topic_type(self):
        scenario_content = """
import osc.ros

scenario test_assert_topic_latency:
    do parallel:
        serial:
            assert_topic_latency(
                topic_name: '/bla',
                latency: 0.5s,
                fail_on_finish: false,
                topic_type: 'std_msgs.msg.String')
            emit end
"""
        parsed_tree = self.parser.parse_input_stream(InputStream(scenario_content))
        model = self.parser.create_internal_model(parsed_tree, "test.osc", False)
        scenarios = create_py_tree(model, self.parser.logger, False)
        self.scenario_execution_ros.scenarios = scenarios
        self.scenario_execution_ros.run()
        self.assertTrue(self.scenario_execution_ros.process_results())


# verifies that the action throws a setup error when an incorrect 'topic_type' is set, ensuring the action's failure.


    def test_with_false_topic_type(self):
        scenario_content = """
import osc.ros

scenario test_assert_topic_latency:
    do parallel:
        serial:
            assert_topic_latency(
                topic_name: '/bla',
                latency: 0.5s,
                fail_on_finish: false,
                topic_type: 'std_msgs.msg.ring')
            emit end
"""
        parsed_tree = self.parser.parse_input_stream(InputStream(scenario_content))
        model = self.parser.create_internal_model(parsed_tree, "test.osc", False)
        scenarios = create_py_tree(model, self.parser.logger, False)
        self.scenario_execution_ros.scenarios = scenarios
        self.scenario_execution_ros.run()
        self.assertFalse(self.scenario_execution_ros.process_results())


# verifies the failure of the action. Since 'fail_on_finish' is true by default, the action fails if the latency exceeds the specified threshold.


    def test_failure(self):
        scenario_content = """
import osc.ros

scenario test_assert_topic_latency:
    do parallel:
        serial:
            assert_topic_latency(
                topic_name: '/bla',
                latency: 0.5s)
            emit end
"""
        parsed_tree = self.parser.parse_input_stream(InputStream(scenario_content))
        model = self.parser.create_internal_model(parsed_tree, "test.osc", False)
        scenarios = create_py_tree(model, self.parser.logger, False)
        self.scenario_execution_ros.scenarios = scenarios
        self.scenario_execution_ros.run()
        self.assertFalse(self.scenario_execution_ros.process_results())

# evaluates the functionality of the action when the latency condition is met, allowing the action to continue running until the scenario ends.
    def test_running(self):
        scenario_content = """
import osc.ros

scenario test_assert_topic_latency:
    do parallel:
        serial:
            assert_topic_latency(
                topic_name: '/bla',
                latency: 1.5s)
            emit end
        serial:
            wait elapsed(10s)
            emit fail
"""
        parsed_tree = self.parser.parse_input_stream(InputStream(scenario_content))
        model = self.parser.create_internal_model(parsed_tree, "test.osc", False)
        scenarios = create_py_tree(model, self.parser.logger, False)
        self.scenario_execution_ros.scenarios = scenarios
        self.scenario_execution_ros.run()
        self.assertFalse(self.scenario_execution_ros.process_results())

# verifies the comparison operator by switching the default value from le to ge. With the latency set higher than the actual latency, the action fails as 'fail_on_finish' is true by default.
    def test_comparison_operator(self):
        scenario_content = """
import osc.ros

scenario test_assert_topic_latency:
    do parallel:
        serial:
            assert_topic_latency(
                topic_name: '/bla',
                latency: 1.5s,
                comparison_operator: comparison_operator!ge)
            emit end
"""
        parsed_tree = self.parser.parse_input_stream(InputStream(scenario_content))
        model = self.parser.create_internal_model(parsed_tree, "test.osc", False)
        scenarios = create_py_tree(model, self.parser.logger, False)
        self.scenario_execution_ros.scenarios = scenarios
        self.scenario_execution_ros.run()
        self.assertFalse(self.scenario_execution_ros.process_results())

    def test_rolling_average_count(self):
        scenario_content = """
import osc.ros

scenario test_assert_topic_latency:
    do parallel:
        serial:
            assert_topic_latency(
                topic_name: '/bla',
                latency: 0.5s,
                rolling_average_count: 5)
            emit end
"""
        parsed_tree = self.parser.parse_input_stream(InputStream(scenario_content))
        model = self.parser.create_internal_model(parsed_tree, "test.osc", False)
        scenarios = create_py_tree(model, self.parser.logger, False)
        self.scenario_execution_ros.scenarios = scenarios
        self.scenario_execution_ros.run()
        self.assertFalse(self.scenario_execution_ros.process_results())

    def test_wait_for_first_message_no_topic_type(self):
        scenario_content = """
import osc.ros

scenario test_assert_topic_latency:
    do parallel:
        serial:
            assert_topic_latency(
                topic_name: '/bla',
                latency: 5s,
                wait_for_first_message: false)
            emit end
"""

        parsed_tree = self.parser.parse_input_stream(InputStream(scenario_content))
        model = self.parser.create_internal_model(parsed_tree, "test.osc", False)
        scenarios = create_py_tree(model, self.parser.logger, False)
        self.scenario_execution_ros.scenarios = scenarios
        self.scenario_execution_ros.run()
        self.assertFalse(self.scenario_execution_ros.process_results())

    def test_wait_for_first_message_with_topic_type(self):
        scenario_content = """
import osc.ros

scenario test_assert_topic_latency:
    do parallel:
        serial: 
            assert_topic_latency(
                topic_name: '/blabla',
                latency: 5s,
                wait_for_first_message: false,
                topic_type: 'std_msgs.msg.String')
            emit end
"""

        parsed_tree = self.parser.parse_input_stream(InputStream(scenario_content))
        model = self.parser.create_internal_model(parsed_tree, "test.osc", False)
        scenarios = create_py_tree(model, self.parser.logger, False)
        self.scenario_execution_ros.scenarios = scenarios
        self.scenario_execution_ros.run()
        self.assertFalse(self.scenario_execution_ros.process_results())

    def test_wait_for_first_message_false_topic_type(self):
        scenario_content = """
import osc.ros

scenario test_assert_topic_latency:
    do parallel:
        serial:
            assert_topic_latency(
                topic_name: '/bla',
                latency: 5s,
                wait_for_first_message: false,
                topic_type: 'std_msgs.msg.ring')
            emit end
"""

        parsed_tree = self.parser.parse_input_stream(InputStream(scenario_content))
        model = self.parser.create_internal_model(parsed_tree, "test.osc", False)
        scenarios = create_py_tree(model, self.parser.logger, False)
        self.scenario_execution_ros.scenarios = scenarios
        self.scenario_execution_ros.run()
        self.assertFalse(self.scenario_execution_ros.process_results())
