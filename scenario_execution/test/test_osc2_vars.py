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

"""
Test parameter parsing
"""
import unittest
import py_trees
from scenario_execution import ScenarioExecution
from scenario_execution.model.osc2_parser import OpenScenario2Parser
from scenario_execution.utils.logging import Logger
from antlr4.InputStream import InputStream


class TestOSC2Parser(unittest.TestCase):
    # pylint: disable=missing-function-docstring, protected-access, no-member, unused-variable, too-many-public-methods
    """
    Unit test for osc2_parser
    """

    def setUp(self) -> None:
        self.parser = OpenScenario2Parser(Logger('test', False))
        self.scenario_execution = ScenarioExecution(debug=False,
                                                    log_model=False,
                                                    live_tree=False,
                                                    scenario_file='test',
                                                    output_dir='')
        self.tree = py_trees.composites.Sequence()

    def parse(self, scenario_content):
        parsed_tree = self.parser.parse_input_stream(InputStream(scenario_content))
        return self.parser.create_internal_model(parsed_tree, self.tree, "test.osc", False)

    def test_var_usage_base_type(self):
        scenario_content = """
scenario test_scenario:
    var test: string = "eins"
    var result: string = test
"""
        model = self.parse(scenario_content)
        test_var = model._ModelElement__children[0]._ModelElement__children[1]
        self.assertEqual("eins", test_var.get_resolved_value())

    def test_var_usage_struct_type_member(self):
        scenario_content = """
struct test_struct:
    param1: string

scenario test_scenario:
    test: test_struct = test_struct("foo")
    var result: string = test.param1
"""
        model = self.parse(scenario_content)
        test_var = model._ModelElement__children[1]._ModelElement__children[1]
        self.assertEqual("foo", test_var.get_resolved_value())


    def test_var_usage_struct_type_member_of_member(self):
        scenario_content = """
struct inner_struct:
    mem1: string = "bar"
    
struct test_struct:
    param1: inner_struct

scenario test_scenario:
    test: test_struct
    var result: string = test.param1.mem1
"""
        model = self.parse(scenario_content)
        test_var = model._ModelElement__children[2]._ModelElement__children[1]
        self.assertEqual("bar", test_var.get_resolved_value())
