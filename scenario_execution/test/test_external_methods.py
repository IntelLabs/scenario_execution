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
import rclpy
from scenario_execution import ScenarioExecution
from scenario_execution.model.osc2_parser import OpenScenario2Parser
from scenario_execution.model.model_to_py_tree import create_py_tree
from scenario_execution.utils.logging import Logger
from antlr4.InputStream import InputStream
from scenario_execution.model.error import OSC2ParsingError


class TestScenarioExecutionSuccess(unittest.TestCase):
    # pylint: disable=missing-function-docstring

    def setUp(self) -> None:
        rclpy.init()
        self.parser = OpenScenario2Parser(Logger('test', False))
        self.scenario_execution = ScenarioExecution(debug=False,
                                                    log_model=False,
                                                    live_tree=False,
                                                    scenario_file='test',
                                                    output_dir='')

    def tearDown(self):
        rclpy.try_shutdown()

    def test_success(self):
        scenario_content = """
import osc.standard.base

struct test:
    def get_float(min_val: float, max_val: float) -> float is external scenario_execution.external_methods.random.get_float()

scenario test_param:
    do serial:
        wait elapsed(test.get_float(min_val: 1, max_val: 3))
"""
        parsed_tree = self.parser.parse_input_stream(InputStream(scenario_content))
        model = self.parser.create_internal_model(parsed_tree, "test.osc", False)
        scenarios = create_py_tree(model, self.parser.logger, False)
        self.scenario_execution.scenarios = scenarios
        self.scenario_execution.run()
        self.assertTrue(self.scenario_execution.process_results())

    def test_not_external(self):
        scenario_content = """
import osc.standard.base

struct test:
    def get_float(min_val: float, max_val: float) -> float is undefined

scenario test_param:
    do serial:
        wait elapsed(test.get_float(min_val: 1, max_val: 3))
"""
        parsed_tree = self.parser.parse_input_stream(InputStream(scenario_content))
        self.assertRaises(ValueError, self.parser.create_internal_model, parsed_tree, "test.osc")

    def test_defined_parameters_differ(self):
        scenario_content = """
import osc.standard.base

struct test:
    def get_float(min_val: float) -> float is external scenario_execution.external_methods.random.get_float()

scenario test_param:
    do serial:
        wait elapsed(test.get_float(min_val: 1))
"""
        parsed_tree = self.parser.parse_input_stream(InputStream(scenario_content))
        model = self.parser.create_internal_model(parsed_tree, "test.osc", False)
        self.assertFalse(self.scenario_execution.process_results())

    def test_call_parameters_differ(self):
        scenario_content = """
import osc.standard.base

struct test:
    def get_float(min_val: float, max_val: float) -> float is external scenario_execution.external_methods.random.get_float()

scenario test_param:
    do serial:
        wait elapsed(test.get_float(min_val: 1))
"""
        parsed_tree = self.parser.parse_input_stream(InputStream(scenario_content))
        model = self.parser.create_internal_model(parsed_tree, "test.osc", False)
        self.assertFalse(self.scenario_execution.process_results())

    def test_call_parameters_differ_2(self):
        scenario_content = """
import osc.standard.base

struct test:
    def get_float(min_val: float, max_val: float) -> float is external scenario_execution.external_methods.random.get_float()

scenario test_param:
    do serial:
        wait elapsed(test.get_float(min_val: 1, UNKNOWN: 3))
"""
        parsed_tree = self.parser.parse_input_stream(InputStream(scenario_content))
        model = self.parser.create_internal_model(parsed_tree, "test.osc", False)
        self.assertFalse(self.scenario_execution.process_results())

    def test_call_parameters_differ_3(self):
        scenario_content = """
import osc.standard.base

struct test:
    def get_float(min_val: float) -> float is external scenario_execution.external_methods.random.get_float()

scenario test_param:
    do serial:
        wait elapsed(test.get_float(min_val: 1, UNKNOWN: 3))
"""
        parsed_tree = self.parser.parse_input_stream(InputStream(scenario_content))
        model = self.parser.create_internal_model(parsed_tree, "test.osc", False)
        self.assertFalse(self.scenario_execution.process_results())
