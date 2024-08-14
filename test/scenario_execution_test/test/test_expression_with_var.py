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
import tempfile
import py_trees
from scenario_execution import ScenarioExecution
from scenario_execution.model.osc2_parser import OpenScenario2Parser
from scenario_execution.model.model_to_py_tree import create_py_tree
from scenario_execution.model.model_blackboard import create_py_tree_blackboard
from scenario_execution.utils.logging import Logger

from antlr4.InputStream import InputStream


class TestCheckData(unittest.TestCase):
    # pylint: disable=missing-function-docstring,missing-class-docstring

    def setUp(self) -> None:
        self.parser = OpenScenario2Parser(Logger('test', False))
        self.scenario_execution = ScenarioExecution(debug=False, log_model=False, live_tree=False,
                                                    scenario_file="test.osc", output_dir=None)
        self.tree = py_trees.composites.Sequence(name="", memory=True)
        self.tmp_file = tempfile.NamedTemporaryFile()

    def execute(self, scenario_content):
        parsed_tree = self.parser.parse_input_stream(InputStream(scenario_content))
        model = self.parser.create_internal_model(parsed_tree, self.tree, "test.osc", False)
        create_py_tree_blackboard(model, self.tree, self.parser.logger, False)
        self.tree = create_py_tree(model, self.tree, self.parser.logger, False)
        self.scenario_execution.tree = self.tree
        self.scenario_execution.run()

    def test_success(self):
        scenario_content = """
import osc.helpers
 
struct current_state:
    var val: int = 1

action set_blackboard_var:
    variable_name: string
    variable_value: string

scenario test_scenario:
    timeout(5s)
    current: current_state
    do parallel:
        serial:
            wait elapsed(0.2s)
            set_blackboard_var("current/val", 2)
            wait elapsed(10s)
        serial:
            wait current.val * 2 + 4 - 4 / 2 == 6
            emit end
"""
        self.execute(scenario_content)
        self.assertTrue(self.scenario_execution.process_results())

    def test_success_not(self):
        scenario_content = """
import osc.helpers
 
struct current_state:
    var val: bool = false
    var val2: bool = false

action set_blackboard_var:
    variable_name: string
    variable_value: string

scenario test_scenario:
    timeout(5s)
    current: current_state
    do parallel:
        serial:
            wait elapsed(0.2s)
            set_blackboard_var("current/val", true)
            wait elapsed(0.2s)
        serial:
            wait current.val and not current.val2
            emit end
"""
        self.execute(scenario_content)
        self.assertTrue(self.scenario_execution.process_results())
