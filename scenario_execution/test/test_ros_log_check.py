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
from scenario_execution_base.model.model_to_py_tree import create_py_tree
from scenario_execution_base.utils.logging import Logger
from antlr4.InputStream import InputStream

os.environ["PYTHONUNBUFFERED"] = '1'


class TestScenarioExectionSuccess(unittest.TestCase):
    # pylint: disable=missing-function-docstring

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.received_msgs = []
        self.node = rclpy.create_node('test_node')

        self.executor = rclpy.executors.MultiThreadedExecutor()
        self.executor.add_node(self.node)
        self.executor_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.executor_thread.start()

        self.callback_group = rclpy.callback_groups.ReentrantCallbackGroup()
        self.srv = self.node.create_timer(1, self.callback)

        self.parser = OpenScenario2Parser(Logger('test'))
        self.scenario_execution = ROSScenarioExecution()

    def tearDown(self):
        self.node.destroy_node()

    def callback(self):
        self.node.get_logger().info("ERROR")

    def test_success(self):
        scenario_content = """
import osc.ros

scenario test_log_check:
    do parallel:
        serial:
            log_check(values: ['ERROR'])
            emit end
        time_out: serial:
            wait elapsed(10s)
            emit fail
"""
        parsed_tree, errors = self.parser.parse_input_stream(InputStream(scenario_content))
        self.assertEqual(errors, 0)
        model = self.parser.create_internal_model(parsed_tree, "test.osc", True)
        self.assertIsNotNone(model)
        scenarios = create_py_tree(model, self.parser.logger, False)
        self.assertIsNotNone(scenarios)
        self.scenario_execution.scenarios = scenarios
        ret = self.scenario_execution.run()
        self.assertTrue(ret)

    def test_timeout(self):
        scenario_content = """
import osc.ros

scenario test_log_check:
    do parallel:
        serial:
            log_check(values: ['UNKNOWN'])
            emit end
        time_out: serial:
            wait elapsed(3s)
            emit fail
"""
        parsed_tree, errors = self.parser.parse_input_stream(InputStream(scenario_content))
        self.assertEqual(errors, 0)
        model = self.parser.create_internal_model(parsed_tree, "test.osc", True)
        self.assertIsNotNone(model)
        scenarios = create_py_tree(model, self.parser.logger, False)
        self.assertIsNotNone(scenarios)
        self.scenario_execution.scenarios = scenarios
        ret = self.scenario_execution.run()
        self.assertFalse(ret)

    def test_module_success(self):
        scenario_content = """
import osc.ros

scenario test_log_check:
    do parallel:
        serial:
            log_check(module_name: 'test_node', values: ['ERROR'])
            emit end
        time_out: serial:
            wait elapsed(3s)
            emit fail
"""
        parsed_tree, errors = self.parser.parse_input_stream(InputStream(scenario_content))
        self.assertEqual(errors, 0)
        model = self.parser.create_internal_model(parsed_tree, "test.osc", True)
        self.assertIsNotNone(model)
        scenarios = create_py_tree(model, self.parser.logger, False)
        self.assertIsNotNone(scenarios)
        self.scenario_execution.scenarios = scenarios
        ret = self.scenario_execution.run()
        self.assertTrue(ret)

    def test_module_timeout(self):
        scenario_content = """
import osc.ros

scenario test_log_check:
    do parallel:
        serial:
            log_check(module_name: 'UNKNOWN', values: ['ERROR'])
            emit end
        time_out: serial:
            wait elapsed(3s)
            emit fail
"""
        parsed_tree, errors = self.parser.parse_input_stream(InputStream(scenario_content))
        self.assertEqual(errors, 0)
        model = self.parser.create_internal_model(parsed_tree, "test.osc", True)
        self.assertIsNotNone(model)
        scenarios = create_py_tree(model, self.parser.logger, False)
        self.assertIsNotNone(scenarios)
        self.scenario_execution.scenarios = scenarios
        ret = self.scenario_execution.run()
        self.assertFalse(ret)
