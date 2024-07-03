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
import py_trees

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
        self.received_msgs = []
        self.node = rclpy.create_node('test_node')
        self.publisher = self.node.create_publisher(String, "/twist", 10)
        self.publish_timer = self.node.create_timer(1, self.publish_messages)
        self.executor = rclpy.executors.MultiThreadedExecutor()
        self.executor.add_node(self.node)
        self.executor_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.executor_thread.start()
        self.tree = py_trees.composites.Sequence()

    def execute(self, scenario_content):
        parsed_tree = self.parser.parse_input_stream(InputStream(scenario_content))
        model = self.parser.create_internal_model(parsed_tree, self.tree, "test.osc", False)
        create_py_tree(model, self.tree, self.parser.logger, False)
        self.scenario_execution_ros.tree = self.tree
        self.scenario_execution_ros.live_tree = True
        self.scenario_execution_ros.run()

    def publish_messages(self):
        msg = String()
        msg.data = 'Hello'
        self.publisher.publish(msg)

    def tearDown(self):
        self.running = False
        self.node.destroy_node()
        rclpy.try_shutdown()


# REQUIRED PARAMETERS
    # topic_name: Name of the topic to test.
    # latency: The acceptable latency in seconds.

# DEFAULT VALUES
    # comparison_operator: 'le' (less than or equal to)
    # fail_on_finish: True
    # rolling_average_count: 1
    # wait_for_first_message: True
    # topic_type: (optional)

# TESTS PERFORMED

# 1. Minimal Test:
    # Description: All default values remain; only topic name and latency are specified.
    # Case 1: Test fails if recorded latency is more than the specified one.
    # Case 2 & 3: Test fails if the topic type is specified and is incorrect or invalid.
    # Case 4: Test succeeds if topic_type is provided and right, but end with failure in this case as recorded latency is greater than expected and fail_on_finish is True (default)
    # Case 5: Test keeps running and ends with timeout if topic provided not exist.
    # Case 6: Test keeps running and ends with a scenario timeout as latency condition is satisfied (recorded latency is less than the specified one).

# 2. Comparison Operator 'ge' (greater than or equal to):
    # Case 7: Test fails if recorded latency is less than the specified one.

# 3. fail_on_finish: False
    # Case 8: Test succeeds if recorded latency is more than the specified one (for default comparison operator 'le').

# 4. rolling_average_count: > 1
    # Case 9: Test fails if recorded latency is more than the specified one (for default comparison operator 'le').

# 5. wait_for_first_message: False
    # Case 10: Test should fail if topic type is not given.
    # Case 11: Test fails if topic type is invalid or wrong.
    # Case 12: Test fails if the first message arrives after the specified latency time.
    # Case 13: Test continues running if message arrives within the latency time.
    # Case 14: Test fails if the topic provided doesn't show up in the latency time.


    def test_case_1(self):
        scenario_content = """
import osc.ros

scenario test_assert_topic_latency:
    do parallel:
        serial:
            assert_topic_latency(
                topic_name: '/twist',
                latency: 0.5s)
            emit end
"""
        self.execute(scenario_content)
        self.assertFalse(self.scenario_execution_ros.process_results())

    def test_case_2(self):
        scenario_content = """
import osc.ros

scenario test_assert_topic_latency:
    do parallel:
        serial:
            assert_topic_latency(
                topic_name: '/twist',
                latency: 0.5s,
                topic_type: 'std_msgs.msg.ring')
            emit end
"""
        self.execute(scenario_content)
        self.assertFalse(self.scenario_execution_ros.process_results())

    def test_case_3(self):
        scenario_content = """
import osc.ros

scenario test_assert_topic_latency:
    do parallel:
        serial:
            assert_topic_latency(
                topic_name: '/twist',
                latency: 0.5s,
                topic_type: 'std_msgs.msg.Bool')
            emit end
"""
        self.execute(scenario_content)
        self.assertFalse(self.scenario_execution_ros.process_results())

    def test_case_4(self):
        scenario_content = """
import osc.ros

scenario test_assert_topic_latency:
    do parallel:
        serial:
            assert_topic_latency(
                topic_name: '/twist',
                latency: 0.5s,
                topic_type: 'std_msgs.msg.String')
            emit end
"""
        self.execute(scenario_content)
        self.assertFalse(self.scenario_execution_ros.process_results())

    def test__case_5(self):
        scenario_content = """
import osc.ros

scenario test_assert_topic_latency:
    do parallel:
        serial:
            assert_topic_latency(
                topic_name: '/twistbla',
                latency: 0.5s)
            emit fail
        time_out: serial:
            wait elapsed(8s)
            emit end
"""
        self.execute(scenario_content)
        self.assertTrue(self.scenario_execution_ros.process_results())

    def test_case_6(self):
        scenario_content = """
import osc.ros

scenario test_assert_topic_latency:
    do parallel:
        serial:
            assert_topic_latency(
                topic_name: '/twist',
                latency: 1.5s)
            emit fail
        serial:
            wait elapsed(8s)
            emit end
"""
        self.execute(scenario_content)
        self.assertTrue(self.scenario_execution_ros.process_results())

    def test_case_7(self):
        scenario_content = """
import osc.ros

scenario test_assert_topic_latency:
    do parallel:
        serial:
            assert_topic_latency(
                topic_name: '/twist',
                latency: 1.5s,
                comparison_operator: comparison_operator!ge)
            emit end
"""
        self.execute(scenario_content)
        self.assertFalse(self.scenario_execution_ros.process_results())

    def test_case_8(self):
        scenario_content = """
import osc.ros

scenario test_assert_topic_latency:
    do parallel:
        serial:
            assert_topic_latency(
                topic_name: '/twist',
                latency: 0.5s,
                fail_on_finish: false)
            emit end
"""
        self.execute(scenario_content)
        self.assertTrue(self.scenario_execution_ros.process_results())

    def test_case_9(self):
        scenario_content = """
import osc.ros

scenario test_assert_topic_latency:
    do parallel:
        serial:
            assert_topic_latency(
                topic_name: '/twist',
                latency: 0.5s,
                rolling_average_count: 5)
            emit end
"""
        self.execute(scenario_content)
        self.assertFalse(self.scenario_execution_ros.process_results())

    def test_case_10(self):
        scenario_content = """
import osc.ros

scenario test_assert_topic_latency:
    do parallel:
        serial:
            assert_topic_latency(
                topic_name: '/twist',
                latency: 0.5s,
                wait_for_first_message: false)
            emit end
"""
        self.execute(scenario_content)
        self.assertFalse(self.scenario_execution_ros.process_results())

    def test_case_11(self):
        scenario_content = """
import osc.ros

scenario test_assert_topic_latency:
    do parallel:
        serial: 
            assert_topic_latency(
                topic_name: '/twist',
                latency: 5s,
                wait_for_first_message: false,
                topic_type: 'std_msgs.msg.Bool')
            emit end
"""
        self.execute(scenario_content)
        self.assertFalse(self.scenario_execution_ros.process_results())

    def test_case_12(self):
        scenario_content = """
import osc.ros

scenario test_assert_topic_latency:
    do parallel:
        serial:
            assert_topic_latency(
                topic_name: '/twist',
                latency: 0.5,
                wait_for_first_message: false,
                topic_type: 'std_msgs.msg.String')
            emit end
"""
        self.execute(scenario_content)
        self.assertFalse(self.scenario_execution_ros.process_results())

    def test_case_13(self):
        scenario_content = """
import osc.ros

scenario test_assert_topic_latency:
    do parallel:
        serial:
            assert_topic_latency(
                topic_name: '/twist',
                latency: 5s,
                wait_for_first_message: false,
                topic_type: 'std_msgs.msg.String')
            emit fail
        serial:
            wait elapsed(10s)
            emit end

"""
        self.execute(scenario_content)
        self.assertTrue(self.scenario_execution_ros.process_results())

    def test_case_14(self):
        scenario_content = """
import osc.ros

scenario test_assert_topic_latency:
    do parallel:
        serial:
            assert_topic_latency(
                topic_name: '/twistbla',
                latency: 5s,
                wait_for_first_message: false,
                topic_type: 'std_msgs.msg.String')
            emit end

"""
        self.execute(scenario_content)
        self.assertFalse(self.scenario_execution_ros.process_results())
