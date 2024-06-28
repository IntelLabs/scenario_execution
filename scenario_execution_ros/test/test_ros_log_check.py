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
import py_trees
import threading
from scenario_execution_ros import ROSScenarioExecution
from scenario_execution.model.osc2_parser import OpenScenario2Parser
from scenario_execution.model.model_to_py_tree import create_py_tree
from scenario_execution.utils.logging import Logger
from antlr4.InputStream import InputStream

os.environ["PYTHONUNBUFFERED"] = '1'


class TestScenarioExectionSuccess(unittest.TestCase):
    # pylint: disable=missing-function-docstring

    def setUp(self):
        rclpy.init()

        self.received_msgs = []
        self.node = rclpy.create_node('test_node_logging')

        self.executor = rclpy.executors.MultiThreadedExecutor()
        self.executor.add_node(self.node)
        self.executor_thread = threading.Thread(target=self.executor.spin)
        self.executor_thread.start()

        self.callback_group = rclpy.callback_groups.ReentrantCallbackGroup()
        self.srv = self.node.create_timer(1, self.callback)

        self.parser = OpenScenario2Parser(Logger('test', False))
        self.scenario_execution_ros = ROSScenarioExecution()
        self.tree = py_trees.composites.Sequence()

    def execute(self, scenario_content):
        parsed_tree = self.parser.parse_input_stream(InputStream(scenario_content))
        model = self.parser.create_internal_model(parsed_tree, self.tree, "test.osc", False)
        scenarios = create_py_tree(model, self.tree, self.parser.logger, False)
        self.scenario_execution_ros.scenarios = scenarios
        self.scenario_execution_ros.run()

    def tearDown(self):
        self.node.destroy_node()
        rclpy.try_shutdown()
        self.executor_thread.join()

    def callback(self):
        self.node.get_logger().info("ERROR")

    def test_success(self):
        scenario_content = """
import osc.ros

scenario test_success:
    do parallel:
        serial:
            log_check(values: ['ERROR'])
            emit end
        time_out: serial:
            wait elapsed(10s)
            emit fail
"""
        self.scenario_execution_ros.live_tree = True
        self.execute(scenario_content)
        self.assertTrue(self.scenario_execution_ros.process_results())

    def test_timeout(self):
        scenario_content = """
import osc.ros

scenario test_timeout:
    do parallel:
        serial:
            log_check(values: ['UNKNOWN'])
            emit end
        time_out: serial:
            wait elapsed(10s)
            emit fail
"""
        self.execute(scenario_content)
        self.assertFalse(self.scenario_execution_ros.process_results())

    def test_module_success(self):
        scenario_content = """
import osc.ros

scenario test_module_success:
    do parallel:
        serial:
            log_check(module_name: 'test_node_logging', values: ['ERROR'])
            emit end
        time_out: serial:
            wait elapsed(10s)
            emit fail
"""
        self.execute(scenario_content)
        self.assertTrue(self.scenario_execution_ros.process_results())

    def test_module_timeout(self):
        scenario_content = """
import osc.ros

scenario test_module_timeout:
    do parallel:
        serial:
            log_check(module_name: 'UNKNOWN', values: ['ERROR'])
            emit end
        time_out: serial:
            wait elapsed(10s)
            emit fail
"""
        self.execute(scenario_content)
        self.assertFalse(self.scenario_execution_ros.process_results())
