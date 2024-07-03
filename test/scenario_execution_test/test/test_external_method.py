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
from scenario_execution.utils.logging import Logger

from antlr4.InputStream import InputStream


class TestCheckData(unittest.TestCase):
    # pylint: disable=missing-function-docstring,missing-class-docstring

    def setUp(self) -> None:
        self.parser = OpenScenario2Parser(Logger('test', False))
        self.scenario_execution = ScenarioExecution(debug=False, log_model=False, live_tree=False,
                                                    scenario_file="test.osc", output_dir=None)
        self.tmp_file = tempfile.NamedTemporaryFile()
        self.tree = py_trees.composites.Sequence()
        self.tmp_file = tempfile.NamedTemporaryFile()

    def execute(self, scenario_content):
        parsed_tree = self.parser.parse_input_stream(InputStream(scenario_content))
        model = self.parser.create_internal_model(parsed_tree, self.tree, "test.osc", True)
        create_py_tree(model, self.tree, self.parser.logger, False)
        self.scenario_execution.tree = self.tree
        self.scenario_execution.live_tree = True
        self.scenario_execution.run()

    def test_success(self):
        scenario_content = """
struct lib:
    def test(n: int, text: string) -> int is external scenario_execution_test.external_methods.external_test.test()
    
action store_action:
    file_path: string
    value: string
    
scenario test:
    do store_action('""" + self.tmp_file.name + """', lib.test(1, "foo"))
"""
        self.execute(scenario_content)
        self.assertTrue(self.scenario_execution.process_results())
        with open(self.tmp_file.name) as f:
            result = f.read()
        self.assertEqual(result, "foo")

    def test_success_named_parameter(self):
        scenario_content = """
struct lib:
    def test(n: int, text: string) -> int is external scenario_execution_test.external_methods.external_test.test()
    
action store_action:
    file_path: string
    value: string
    
scenario test:
    do store_action('""" + self.tmp_file.name + """', lib.test(n: 1, text: "foo"))
"""
        self.execute(scenario_content)
        self.assertTrue(self.scenario_execution.process_results())
        with open(self.tmp_file.name) as f:
            result = f.read()
        self.assertEqual(result, "foo")

    def test_fail_wrong_named_parameter(self):
        scenario_content = """
struct lib:
    def test(n: int, text: string) -> int is external scenario_execution_test.external_methods.external_test.test()
    
action store_action:
    file_path: string
    value: string
    
scenario test:
    do store_action('""" + self.tmp_file.name + """', lib.test(n: 1, UNKNOWN: "foo"))
"""
        parsed_tree = self.parser.parse_input_stream(InputStream(scenario_content))
        model = self.parser.create_internal_model(parsed_tree, self.tree, "test.osc", False)
        create_py_tree(model, self.tree, self.parser.logger, False)
        self.scenario_execution.tree = self.tree
        self.assertRaises(ValueError, self.scenario_execution.run)


    def test_success_use_default_value(self):
        scenario_content = """
struct lib:
    def test(n: int = 10, text: string = "foo") -> int is external scenario_execution_test.external_methods.external_test.test()
    
action store_action:
    file_path: string
    value: string
    
scenario test:
    do store_action('""" + self.tmp_file.name + """', lib.test())
"""
        self.execute(scenario_content)
        self.assertTrue(self.scenario_execution.process_results())
        with open(self.tmp_file.name) as f:
            result = f.read()
        self.assertEqual(result, "foo")


    def test_fail_method_exception(self):
        scenario_content = """
struct lib:
    def test(n: int, text: string) -> int is external scenario_execution_test.external_methods.external_test.test()
    
action store_action:
    file_path: string
    value: string
    
scenario test:
    do store_action('""" + self.tmp_file.name + """', lib.test(99, "foo"))
"""
        parsed_tree = self.parser.parse_input_stream(InputStream(scenario_content))
        model = self.parser.create_internal_model(parsed_tree, self.tree, "test.osc", False)
        create_py_tree(model, self.tree, self.parser.logger, False)
        self.scenario_execution.tree = self.tree
        self.assertRaises(ValueError, self.scenario_execution.run)


    def test_success_struct(self):
        scenario_content = """
struct test_struct:
    mem1: int = 3
    mem2: string = "bar"
    
struct lib:
    def test_dict(val: test_struct) -> int is external scenario_execution_test.external_methods.external_test.test_dict()
    
action store_action:
    file_path: string
    value: string
    
scenario test:
    do store_action('""" + self.tmp_file.name + """', lib.test_dict(test_struct(mem1: 1, mem2: "bar")))
"""
        self.execute(scenario_content)
        self.assertTrue(self.scenario_execution.process_results())
        with open(self.tmp_file.name) as f:
            result = f.read()
        self.assertEqual(result, "{'mem1': 1, 'mem2': 'bar'}")
