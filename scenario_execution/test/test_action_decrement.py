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
from scenario_execution.actions.base_action import ActionError
from scenario_execution import ScenarioExecution
from scenario_execution.model.osc2_parser import OpenScenario2Parser
from scenario_execution.model.model_to_py_tree import create_py_tree
from scenario_execution.model.model_blackboard import create_py_tree_blackboard
from scenario_execution.utils.logging import Logger
from antlr4.InputStream import InputStream


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
        create_py_tree_blackboard(model, self.tree, self.parser.logger, False)
        self.tree = create_py_tree(model, self.tree, self.parser.logger, False)
        self.scenario_execution.tree = self.tree
        self.scenario_execution.run()

    def test_fail_non_variable(self):
        scenario_content = """
import osc.helpers
    
scenario test:
    timeout(3s)
    my_var: int = 0
    do serial:
        decrement(my_var)
"""
        self.assertRaises(ActionError, self.execute, scenario_content)
        self.assertFalse(self.scenario_execution.process_results())

    def test_fail_unknown_variable(self):
        scenario_content = """
import osc.helpers
    
scenario test:
    timeout(3s)
    do serial:
        decrement(UNKNOWN)
"""
        self.assertRaises(ValueError, self.execute, scenario_content)

    def test_success_direct_var(self):
        scenario_content = """
import osc.helpers
    
scenario test:
    timeout(5s)
    var my_var: int = 1
    do parallel:
        serial:
            decrement(my_var)
            decrement(my_var)
            wait my_var == -2
        serial:
            wait elapsed(3s)
            decrement(my_var)
"""
        self.execute(scenario_content)
        self.assertTrue(self.scenario_execution.process_results())

    def test_fail_actor_non_variable(self):
        scenario_content = """
import osc.helpers
    
actor test_actor:
    my_var: int = 42
    
scenario test:
    timeout(3s)
    my_actor: test_actor
    do serial:
        decrement(my_actor.my_var)
"""
        self.assertRaises(ActionError, self.execute, scenario_content)
        self.assertFalse(self.scenario_execution.process_results())

    def test_success_actor_var(self):
        scenario_content = """
import osc.helpers

actor test_actor:
    var my_var: int = 0
    
scenario test:
    timeout(5s)
    my_actor: test_actor
    do parallel:
        serial:
            decrement(my_actor.my_var)
            decrement(my_actor.my_var)
            wait my_actor.my_var == -3
        serial:
            wait elapsed(3s)
            decrement(my_actor.my_var)
"""
        self.execute(scenario_content)
        self.assertTrue(self.scenario_execution.process_results())

    def test_success_var_in_actor_param_repeat(self):
        scenario_content = """
import osc.helpers

struct test_stru:
    var my_sub_var: int = 16

actor test_actor:
    my_struct: test_stru
    
scenario test_scenario:
    timeout(10s)
    my_actor: test_actor
    do parallel:
        serial:
            repeat()
            decrement(my_actor.my_struct.my_sub_var)
            wait elapsed(0.5s)
        serial:
            wait my_actor.my_struct.my_sub_var == 10
            emit end
"""
        self.execute(scenario_content)
        self.assertTrue(self.scenario_execution.process_results())
