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
import py_trees
import threading
import rclpy
from rclpy.lifecycle import LifecycleNode
from scenario_execution_ros import ROSScenarioExecution
from scenario_execution.model.osc2_parser import OpenScenario2Parser
from scenario_execution.model.model_to_py_tree import create_py_tree
from scenario_execution.utils.logging import Logger
from antlr4.InputStream import InputStream
import subprocess  # nosec B404
import time


class TestScenarioExectionSuccess(unittest.TestCase):
    # pylint: disable=missing-function-docstring

    def setUp(self) -> None:
        rclpy.init()
        self.running = True
        self.parser = OpenScenario2Parser(Logger('test', False))
        self.scenario_execution_ros = ROSScenarioExecution()
        self.node = LifecycleNode('test_lifecycle_node')
        self.dynamic_node = LifecycleNode('test_lifecycle_dynamic_node')
        self.executor = rclpy.executors.MultiThreadedExecutor()
        self.executor.add_node(self.node)
        self.executor.add_node(self.dynamic_node)
        self.executor_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.executor_thread.start()
        self.tree = py_trees.composites.Sequence()

    def execute(self, scenario_content):
        parsed_tree = self.parser.parse_input_stream(InputStream(scenario_content))
        model = self.parser.create_internal_model(parsed_tree, self.tree, "test.osc", False)
        create_py_tree(model, self.tree, self.parser.logger, False)
        self.scenario_execution_ros.tree = self.tree
        self.scenario_execution_ros.run()

    def tearDown(self):
        self.running = False
        self.node.destroy_node()
        self.node.destroy_node()
        rclpy.try_shutdown()


# REQUIRED PARAMETERS
    # node_name: Name of the topic to test.
    # state_sequence: list of state_sequence ['unconfigured', 'inactive', 'active', 'finalized'].

# DEFAULT VALUES
    # allow_initial_skip: False
    # fail_on_unexpected: True

# TESTS PERFORMED

# 1. Minimal Test:
    # Description: All default values remain; only Node name and state are specified.
    # Case 1: Test fails if state of the node doesn't match the specified state at index 0.
    # Case 2: Test keeps running and ends with scenario or timeout if state maches the specified state.
    # Case 3: Test keeps running and ends with scenario or timeout if specified lifecycle node is not found.
    # Case 4: Test fails if the specified state are not in the default list.

# 3. allow_initial_skip: False
    # Case 5: Test succeeds if state of the node doesn't match the specified state.

# 4. allow_initial_skip: True
    # Case 6: Test keeps running and ends with scenario or timeout if start state maches any specified state in the list.
    # Case 7: Test fails if state of the node doesn't match any specified state in the list.
    # Case 8: Test keeps running and ends with scenario or timeout if the node transition through the specified states in the correct order.


    def test_case_1(self):
        scenario_content = """
import osc.ros

scenario test_assert_lifecycle_state:
        do serial:
            assert_lifecycle_state(
                node_name: 'test_lifecycle_node',
                state_sequence: [lifecycle_state!inactive])
            emit end
"""
        self.execute(scenario_content)
        self.assertFalse(self.scenario_execution_ros.process_results())

    def test_case_2(self):
        scenario_content = """
import osc.ros

scenario test_assert_lifecycle_state:
    do parallel:
        serial:
            assert_lifecycle_state(
                node_name: 'test_lifecycle_node',
                state_sequence: [lifecycle_state!unconfigured])
            emit fail
        time_out: serial:
            wait elapsed(8s)
            emit end
"""
        self.execute(scenario_content)
        self.assertTrue(self.scenario_execution_ros.process_results())

    def test_case_3(self):
        scenario_content = """
import osc.ros

scenario test_assert_lifecycle_state:
    do parallel:
        serial:
            assert_lifecycle_state(
                node_name: 'test_node',
                state_sequence: [lifecycle_state!unconfigured])
            emit end
        time_out: serial:
            wait elapsed(8s)
            emit fail
"""
        self.execute(scenario_content)
        self.assertFalse(self.scenario_execution_ros.process_results())

    def test_case_4(self):
        scenario_content = """
import osc.ros

scenario test_assert_lifecycle_state:
        do serial:
            assert_lifecycle_state(
                node_name: 'test_lifecycle_node',
                state_sequence: ['inact'],
                allow_initial_skip: false)
            emit end
"""
        self.execute(scenario_content)
        self.assertFalse(self.scenario_execution_ros.process_results())

    def test_case_5(self):
        scenario_content = """
import osc.ros

scenario test_assert_lifecycle_state:
        do serial:
            assert_lifecycle_state(
                node_name: 'test_lifecycle_node',
                state_sequence: [lifecycle_state!unconfigured],
                allow_initial_skip: false,
                keep_running: false)
            emit end
"""
        self.execute(scenario_content)
        self.assertTrue(self.scenario_execution_ros.process_results())

    def test_case_6(self):
        scenario_content = """
import osc.ros

scenario test_assert_lifecycle_state:
    do parallel:
        serial:
            assert_lifecycle_state(
                node_name: 'test_lifecycle_node',
                state_sequence: [lifecycle_state!inactive, lifecycle_state!unconfigured, lifecycle_state!active],
                allow_initial_skip: true )
            emit fail
        time_out: serial:
            wait elapsed(8s)
            emit end
"""
        self.execute(scenario_content)
        self.assertTrue(self.scenario_execution_ros.process_results())

    def test_case_7(self):
        scenario_content = """
import osc.ros

scenario test_assert_lifecycle_state:
    do parallel:
        serial:
            assert_lifecycle_state(
                node_name: 'test_lifecycle_node',
                state_sequence: [lifecycle_state!inactive, lifecycle_state!active],
                allow_initial_skip: true )
            emit fail
        time_out: serial:
            wait elapsed(8s)
            emit end
"""
        self.execute(scenario_content)
        self.assertFalse(self.scenario_execution_ros.process_results())

    def test_case_8(self):
        scenario_content = """
import osc.ros

scenario test_assert_lifecycle_state:
    do parallel:
        serial:
            assert_lifecycle_state(
                node_name: 'test_lifecycle_dynamic_node',
                state_sequence: [lifecycle_state!unconfigured, lifecycle_state!inactive, lifecycle_state!active],
                allow_initial_skip: true )
            emit fail
        time_out: serial:
            wait elapsed(20s)
            emit end
"""
        thread_change_state = threading.Thread(target=change_node_state)
        thread_change_state.start()
        self.execute(scenario_content)
        self.assertTrue(self.scenario_execution_ros.process_results())


def change_node_state():
    time.sleep(5)
    subprocess.run(['ros2', 'lifecycle', 'set', '/test_lifecycle_dynamic_node', 'configure'], stdout=subprocess.PIPE, timeout=5, check=True)
    print("The node has successfully transitioned to the 'inactive' state.")
    time.sleep(5)
    subprocess.run(['ros2', 'lifecycle', 'set', '/test_lifecycle_dynamic_node', 'activate'], stdout=subprocess.PIPE, timeout=5, check=True)
    print("The node has successfully transitioned to the 'active' state.")
