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
from scenario_execution import ScenarioExecution
from scenario_execution.model.osc2_parser import OpenScenario2Parser
from scenario_execution.model.model_to_py_tree import create_py_tree
from scenario_execution.utils.logging import Logger
from antlr4.InputStream import InputStream
from datetime import datetime


class TestScenarioExecutionSuccess(unittest.TestCase):
    # pylint: disable=missing-function-docstring

    def setUp(self) -> None:
        self.parser = OpenScenario2Parser(Logger('test', False))
        self.scenario_execution = ScenarioExecution(debug=False,
                                                    log_model=False,
                                                    live_tree=False,
                                                    scenario_file='test',
                                                    output_dir='')
        self.tree = py_trees.composites.Sequence(name="", memory=True)

    def execute(self, scenario_content):
        parsed_tree = self.parser.parse_input_stream(InputStream(scenario_content))
        model = self.parser.create_internal_model(parsed_tree, self.tree, "test.osc", False)
        create_py_tree(model, self.tree, self.parser.logger, False)
        self.scenario_execution.tree = self.tree
        self.scenario_execution.run()

    def test_failure(self):
        scenario_content = """
import osc.standard.base
import osc.helpers

scenario test_run_process:
    timeout(10s)
    do serial:
        run_process() with:
            keep(it.command == 'false')
"""
        self.execute(scenario_content)
        self.assertFalse(self.scenario_execution.process_results())

    def test_success(self):
        scenario_content = """
import osc.standard.base
import osc.helpers

scenario test_run_process:
    timeout(10s)
    do serial:
        run_process() with:
            keep(it.command == 'true')
"""
        self.execute(scenario_content)
        self.assertTrue(self.scenario_execution.process_results())

    def test_multi_element_command(self):
        scenario_content = """
import osc.standard.base
import osc.helpers

scenario test_run_process:
    timeout(10s)
    do serial:
        run_process('sleep 2')
"""
        self.execute(scenario_content)
        self.assertTrue(self.scenario_execution.process_results())

    def test_wait_for_shutdown_false(self):
        scenario_content = """
import osc.standard.base
import osc.helpers

scenario test_run_process:
    do serial:
        run_process('sleep 15', wait_for_shutdown: false)
"""
        parsed_tree = self.parser.parse_input_stream(InputStream(scenario_content))
        model = self.parser.create_internal_model(parsed_tree, self.tree, "test.osc", False)
        create_py_tree(model, self.tree, self.parser.logger, False)
        self.scenario_execution.tree = self.tree

        start = datetime.now()
        self.scenario_execution.run()
        end = datetime.now()
        duration = (end-start).total_seconds()
        self.assertLessEqual(duration, 10.)
        self.assertTrue(self.scenario_execution.process_results())

    def test_signal_parsing(self):
        scenario_content = """
import osc.standard.base
import osc.helpers

scenario test_run_process:
    do run_process('sleep 15', wait_for_shutdown: false, shutdown_signal: signal!sigint)
"""
        parsed_tree = self.parser.parse_input_stream(InputStream(scenario_content))
        model = self.parser.create_internal_model(parsed_tree, self.tree, "test.osc", False)
        create_py_tree(model, self.tree, self.parser.logger, False)
