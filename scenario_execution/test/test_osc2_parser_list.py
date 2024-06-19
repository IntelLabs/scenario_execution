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
Test list parsing
"""
import unittest

from scenario_execution.model.osc2_parser import OpenScenario2Parser
from scenario_execution.utils.logging import Logger
from antlr4.InputStream import InputStream


class TestOSC2Parser(unittest.TestCase):
    # pylint: disable=missing-function-docstring, protected-access, no-member, unused-variable
    """
    Unit test for osc2_parser
    """

    def setUp(self) -> None:
        self.parser = OpenScenario2Parser(Logger('test', False))

    def test_invalid_listof(self):
        scenario_content = """
struct listoffoo

global test: listoffoo
"""
        parsed_tree = self.parser.parse_input_stream(InputStream(scenario_content))
        self.assertRaises(ValueError, self.parser.create_internal_model, parsed_tree, "test.osc")

    def test_string_list(self):
        scenario_content = """
global test: list of string = ["foo", "bar"]
"""
        parsed_tree = self.parser.parse_input_stream(InputStream(scenario_content))
        model = self.parser.create_internal_model(parsed_tree, "test.osc", False)

        test_list = model._ModelElement__children[0].get_resolved_value()
        self.assertEqual(["foo", "bar"], test_list)

    def test_empty_list(self):
        scenario_content = """
global test: list of string = [ ]
"""
        self.assertRaises(ValueError, self.parser.parse_input_stream, InputStream(scenario_content))

    def test_mixed_basetypes_list(self):
        scenario_content = """
global test: list of string = ["foo", 3]
"""
        parsed_tree = self.parser.parse_input_stream(InputStream(scenario_content))
        self.assertRaises(ValueError, self.parser.create_internal_model, parsed_tree, "test.osc")

    def test_struct_list(self):
        scenario_content = """
struct test_struct:
    member: string
global test: list of test_struct = [test_struct('val1'), test_struct('val2')]
"""
        parsed_tree = self.parser.parse_input_stream(InputStream(scenario_content))
        model = self.parser.create_internal_model(parsed_tree, "test.osc", False)

        test_list = model._ModelElement__children[1].get_resolved_value()
        self.assertEqual([{'member': 'val1'}, {'member': 'val2'}], test_list)

    def test_different_structs_in_list(self):
        scenario_content = """
struct test_struct:
    member: string
struct second_struct:
    member2: string
global test: list of test_struct = [test_struct('val1'), second_struct('invalid')]
"""
        parsed_tree = self.parser.parse_input_stream(InputStream(scenario_content))
        self.assertRaises(ValueError, self.parser.create_internal_model, parsed_tree, "test.osc")

    def test_assign_struct_list(self):
        scenario_content = """
struct test_struct:
    member: string

global test1: list of test_struct = [test_struct('val1'), test_struct('val2')]
global test2: list of test_struct = test1
"""
        parsed_tree = self.parser.parse_input_stream(InputStream(scenario_content))
        model = self.parser.create_internal_model(parsed_tree, "test.osc", False)

        test1 = model._ModelElement__children[1].get_resolved_value()
        test2 = model._ModelElement__children[2].get_resolved_value()
        self.assertEqual([{'member': 'val1'}, {'member': 'val2'}], test1)
        self.assertEqual([{'member': 'val1'}, {'member': 'val2'}], test2)

    def test_assign_basetype_list(self):
        scenario_content = """
global test1: list of float = [2.1, 4.3]
global test2: list of float = test1
"""
        parsed_tree = self.parser.parse_input_stream(InputStream(scenario_content))
        model = self.parser.create_internal_model(parsed_tree, "test.osc", False)

        test1 = model._ModelElement__children[0].get_resolved_value()
        test2 = model._ModelElement__children[1].get_resolved_value()
        self.assertEqual([2.1, 4.3], test1)
        self.assertEqual([2.1, 4.3], test2)

    def test_assign_wrong_basetype_list(self):
        scenario_content = """
global test1: list of float = [2.1, 4.3]
global test2: list of string = test1
"""
        parsed_tree = self.parser.parse_input_stream(InputStream(scenario_content))
        self.assertRaises(ValueError, self.parser.create_internal_model, parsed_tree, "test.osc")
