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
Test for osc2_parser
"""
import unittest

from scenario_execution_base.model.osc2_parser import OpenScenario2Parser
from scenario_execution_base.utils.logging import Logger
from antlr4.InputStream import InputStream


class TestOSC2Parser(unittest.TestCase):
    # pylint: disable=missing-function-docstring, protected-access, no-member, unused-variable
    """
    Unit test for osc2_parser
    """

    def setUp(self) -> None:
        self.parser = OpenScenario2Parser(Logger('test'))

    def test_enum_ref(self):
        scenario_content = """
enum test_enum: [
    val1,
    val2
]
struct test:
    param1: test_enum = test_enum!val1
"""
        parsed_tree, errors = self.parser.parse_input_stream(InputStream(scenario_content))
        self.assertEqual(errors, 0)
        model = self.parser.create_internal_model(parsed_tree, "test.osc", True)
        self.assertIsNotNone(model)
        test_struct = model._ModelElement__children[1]
        self.assertEqual(test_struct.get_resolved_value(), {'param1': ('val1', 0)})

    def test_enum_ref_invalid(self):
        scenario_content = """
enum test_enum: [
    val1,
    val2
]
struct test:
    param1: test_enum = test_enum!UNKNOWN
"""
        parsed_tree, errors = self.parser.parse_input_stream(InputStream(scenario_content))
        self.assertEqual(errors, 0)
        model = self.parser.create_internal_model(parsed_tree, "test.osc")
        self.assertIsNone(model)

    def test_enum_ref_invalid_type(self):
        scenario_content = """
enum test_enum: [
    val1,
    val2
]
struct test:
    param1: test_enum = UNKNOWN!val1
"""
        parsed_tree, errors = self.parser.parse_input_stream(InputStream(scenario_content))
        self.assertEqual(errors, 0)
        model = self.parser.create_internal_model(parsed_tree, "test.osc")
        self.assertIsNone(model)

    def test_enum_ref_other_enum_type(self):
        scenario_content = """
enum other_enum: [
    val1,
    val2
]

enum test_enum: [
    val1,
    val2
]

struct test:
    param1: test_enum = other_enum!val1
"""
        parsed_tree, errors = self.parser.parse_input_stream(InputStream(scenario_content))
        self.assertEqual(errors, 0)
        model = self.parser.create_internal_model(parsed_tree, "test.osc")
        self.assertIsNone(model)

    def test_enum_val_non_numeric(self):
        scenario_content = """
enum test_enum: [
    val1,
    val2
]

struct test:
    param1: test_enum = test_enum!val1
    param2: test_enum = test_enum!val2
"""
        parsed_tree, errors = self.parser.parse_input_stream(InputStream(scenario_content))
        self.assertEqual(errors, 0)
        model = self.parser.create_internal_model(parsed_tree, "test.osc", True)
        self.assertIsNotNone(model)
        test_struct = model._ModelElement__children[1]
        self.assertEqual(test_struct.get_resolved_value(), {'param1': ('val1', 0), 'param2': ('val2', 1)})

    def test_enum_partially_numeric(self):
        scenario_content = """
enum test_enum: [
    val1 = 4,
    val2
]

struct test:
    param1: test_enum = test_enum!val1
    param2: test_enum = test_enum!val2
"""
        parsed_tree, errors = self.parser.parse_input_stream(InputStream(scenario_content))
        self.assertEqual(errors, 0)
        model = self.parser.create_internal_model(parsed_tree, "test.osc", True)
        self.assertIsNotNone(model)
        test_struct = model._ModelElement__children[1]
        self.assertEqual(test_struct.get_resolved_value(), {'param1': ('val1', 4), 'param2': ('val2', 5)})

    def test_enum_fully_numeric(self):
        scenario_content = """
enum test_enum: [
    val1 = 4,
    val2 = 19
]

struct test:
    param1: test_enum = test_enum!val1
    param2: test_enum = test_enum!val2
"""
        parsed_tree, errors = self.parser.parse_input_stream(InputStream(scenario_content))
        self.assertEqual(errors, 0)
        model = self.parser.create_internal_model(parsed_tree, "test.osc", True)
        self.assertIsNotNone(model)
        test_struct = model._ModelElement__children[1]
        self.assertEqual(test_struct.get_resolved_value(), {'param1': ('val1', 4), 'param2': ('val2', 19)})
