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
from datetime import datetime
from scenario_execution.scenario_execution_base import ScenarioExecution
from scenario_execution.model.osc2_parser import OpenScenario2Parser
from scenario_execution.model.model_to_py_tree import create_py_tree
from .common import DebugLogger
from antlr4.InputStream import InputStream


class TestExternalMethodsRandom(unittest.TestCase):
    # pylint: disable=missing-function-docstring

    def setUp(self) -> None:
        self.logger = DebugLogger("")
        self.parser = OpenScenario2Parser(self.logger)
        self.tree = py_trees.composites.Sequence(name="", memory=True)
        self.scenario_execution = ScenarioExecution(debug=False,
                                                    log_model=False,
                                                    live_tree=False,
                                                    scenario_file='test',
                                                    output_dir='', logger=self.logger)
        self.tree = py_trees.composites.Sequence(name="", memory=True)

    def execute(self, scenario_content):
        parsed_tree = self.parser.parse_input_stream(InputStream(scenario_content))
        model = self.parser.create_internal_model(parsed_tree, self.tree, "test.osc", False)
        self.tree = create_py_tree(model, self.tree, self.parser.logger, False)
        self.scenario_execution.tree = self.tree
        self.scenario_execution.run()

    def test_get_random_int(self):
        scenario_content = """
import osc.helpers

scenario test_success:
    do serial:
        wait elapsed(random.get_int(0, 10))
        emit end
"""
        parsed_tree = self.parser.parse_input_stream(InputStream(scenario_content))
        model = self.parser.create_internal_model(parsed_tree, self.tree, "test.osc", False)
        self.tree = create_py_tree(model, self.tree, self.parser.logger, False)
        self.scenario_execution.tree = self.tree

        start_time = datetime.now()
        self.scenario_execution.run()
        end_time = datetime.now()
        self.assertTrue(self.scenario_execution.process_results())

        delta = end_time - start_time
        self.assertLess(delta.total_seconds(), 10.)

    def test_get_random_string(self):
        scenario_content = """
import osc.helpers

scenario test_success:
    do serial:
        log(random.get_random_string(["test", "test-scenario", "scenario-test"]))
        emit end
"""
        self.execute(scenario_content)
        self.assertTrue(self.scenario_execution.process_results())
        valid_strings = ["test", "test-scenario", "scenario-test"]
        log_messages = self.logger.logs_info
        for log_message in log_messages:
            if log_message in valid_strings:
                self.assertIn(log_message, valid_strings)
