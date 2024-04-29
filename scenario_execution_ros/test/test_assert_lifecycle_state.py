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
from rclpy.lifecycle import LifecycleNode
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
        self.node = LifecycleNode('test_lifecycle_node')
        self.executor = rclpy.executors.MultiThreadedExecutor()
        self.executor.add_node(self.node)
        self.executor_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.executor_thread.start()

    def tearDown(self):
        self.running = False
        self.node.destroy_node()
        rclpy.try_shutdown()


# REQUIRED PARAMETERS
    # node_name: Name of the topic to test.
    # states: list of states ['unconfigured', 'inactive', 'active', 'finalized'].

# DEFAULT VALUES
    # allow_inital_state_skip: False
    # fail_on_finish: True

# TESTS PERFORMED

# 1. Minimal Test:
    # Description: All default values remain; only Node name and state are specified.
    # Case 1: Test fails if state of the node doesn't match the specified state at index 0.
    # Case 2: Test keeps running and ends with scenario or timeout if state maches the specified state.
    # Case 3: Test keeps running and ends with scenario or timeout if specified lifecycle node is not found.
    # Case 4: Test fails if the specified states are not in the defaul list.

# 3. fail_on_finish: False
    # Case 5: Test succeeds if state of the node doesn't match the specified state.

# 4. allow_inital_state_true: True
    # Case 6: Test keeps running and ends with scenario or timeout if start state maches any specified state in the list.
    # Case 7: Test fails if state of the node doesn't match any specified state in the list.


    def test_case_1(self):
        scenario_content = """
import osc.ros

scenario test_assert_lifecycle_state:
        do serial:
            assert_lifecycle_state(
                node_name: 'test_lifecycle_node',
                states: ['inactive'])
            emit end
"""
        parsed_tree = self.parser.parse_input_stream(InputStream(scenario_content))
        model = self.parser.create_internal_model(parsed_tree, "test.osc", False)
        scenarios = create_py_tree(model, self.parser.logger, False)
        self.scenario_execution_ros.scenarios = scenarios
        self.scenario_execution_ros.run()
        self.assertFalse(self.scenario_execution_ros.process_results())

    def test_case_2(self):
        scenario_content = """
import osc.ros

scenario test_assert_lifecycle_state:
    do parallel:
        serial:
            assert_lifecycle_state(
                node_name: 'test_lifecycle_node',
                states: ['unconfigured'])
            emit fail
        time_out: serial:
            wait elapsed(8s)
            emit end
"""
        parsed_tree = self.parser.parse_input_stream(InputStream(scenario_content))
        model = self.parser.create_internal_model(parsed_tree, "test.osc", False)
        scenarios = create_py_tree(model, self.parser.logger, False)
        self.scenario_execution_ros.scenarios = scenarios
        self.scenario_execution_ros.run()
        self.assertTrue(self.scenario_execution_ros.process_results())

    def test_case_3(self):
        scenario_content = """
import osc.ros

scenario test_assert_lifecycle_state:
    do parallel:
        serial:
            assert_lifecycle_state(
                node_name: 'test_node',
                states: ['unconfigured'])
            emit end
        time_out: serial:
            wait elapsed(8s)
            emit fail
"""
        parsed_tree = self.parser.parse_input_stream(InputStream(scenario_content))
        model = self.parser.create_internal_model(parsed_tree, "test.osc", False)
        scenarios = create_py_tree(model, self.parser.logger, False)
        self.scenario_execution_ros.scenarios = scenarios
        self.scenario_execution_ros.run()
        self.assertFalse(self.scenario_execution_ros.process_results())

    def test_case_4(self):
        scenario_content = """
import osc.ros

scenario test_assert_lifecycle_state:
        do serial:
            assert_lifecycle_state(
                node_name: 'test_lifecycle_node',
                states: ['inact'],
                fail_on_finish: false)
            emit end
"""
        parsed_tree = self.parser.parse_input_stream(InputStream(scenario_content))
        model = self.parser.create_internal_model(parsed_tree, "test.osc", False)
        scenarios = create_py_tree(model, self.parser.logger, False)
        self.scenario_execution_ros.scenarios = scenarios
        self.scenario_execution_ros.run()
        self.assertFalse(self.scenario_execution_ros.process_results())

    def test_case_5(self):
        scenario_content = """
import osc.ros

scenario test_assert_lifecycle_state:
        do serial:
            assert_lifecycle_state(
                node_name: 'test_lifecycle_node',
                states: ['inactive'],
                fail_on_finish: false)
            emit end
"""
        parsed_tree = self.parser.parse_input_stream(InputStream(scenario_content))
        model = self.parser.create_internal_model(parsed_tree, "test.osc", False)
        scenarios = create_py_tree(model, self.parser.logger, False)
        self.scenario_execution_ros.scenarios = scenarios
        self.scenario_execution_ros.run()
        self.assertTrue(self.scenario_execution_ros.process_results())

    def test_case_6(self):
        scenario_content = """
import osc.ros

scenario test_assert_lifecycle_state:
    do parallel:
        serial:
            assert_lifecycle_state(
                node_name: 'test_lifecycle_node',
                states: ['inactive', 'unconfigured', 'active'],
                allow_inital_state_skip: true )
            emit fail
        time_out: serial:
            wait elapsed(8s)
            emit end
"""
        parsed_tree = self.parser.parse_input_stream(InputStream(scenario_content))
        model = self.parser.create_internal_model(parsed_tree, "test.osc", False)
        scenarios = create_py_tree(model, self.parser.logger, False)
        self.scenario_execution_ros.scenarios = scenarios
        self.scenario_execution_ros.run()
        self.assertTrue(self.scenario_execution_ros.process_results())

    def test_case_7(self):
        scenario_content = """
import osc.ros

scenario test_assert_lifecycle_state:
    do parallel:
        serial:
            assert_lifecycle_state(
                node_name: 'test_lifecycle_node',
                states: ['inactive', 'active'],
                allow_inital_state_skip: true )
            emit fail
        time_out: serial:
            wait elapsed(8s)
            emit end
"""
        parsed_tree = self.parser.parse_input_stream(InputStream(scenario_content))
        model = self.parser.create_internal_model(parsed_tree, "test.osc", False)
        scenarios = create_py_tree(model, self.parser.logger, False)
        self.scenario_execution_ros.scenarios = scenarios
        self.scenario_execution_ros.run()
        self.assertFalse(self.scenario_execution_ros.process_results())
