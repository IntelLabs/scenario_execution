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
from scenario_execution import ROSScenarioExecution
from scenario_execution_base.model.osc2_parser import OpenScenario2Parser
from scenario_execution_base.model.model_to_py_tree import create_py_tree
from scenario_execution_base.utils.logging import Logger
from antlr4.InputStream import InputStream


class TestScenarioExecutionSuccess(unittest.TestCase):
    # pylint: disable=missing-function-docstring

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self) -> None:
        self.parser = OpenScenario2Parser(Logger('test'))
        self.scenario_execution = ROSScenarioExecution()

    def test_failure(self):
        scenario_content = """
import osc.standard
import osc.helpers

scenario test_run_process:
    do parallel:
        serial:
            run_process() with:
                keep(it.command == 'false')
            emit end
        time_out: serial:
            wait elapsed(10s)
            time_out_shutdown: emit fail
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

    def test_success(self):
        scenario_content = """
import osc.standard
import osc.helpers

scenario test_run_process:
    do parallel:
        serial:
            run_process() with:
                keep(it.command == 'true')
            emit end
        time_out: serial:
            wait elapsed(10s)
            time_out_shutdown: emit fail
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

    def test_multi_element_command(self):
        scenario_content = """
import osc.standard
import osc.helpers

scenario test_run_process:
    do parallel:
        serial:
            run_process('sleep 2')
            emit end
        time_out: serial:
            wait elapsed(10s)
            time_out_shutdown: emit fail
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
