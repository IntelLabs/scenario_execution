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

from scenario_execution.model.osc2_parser import OpenScenario2Parser
from scenario_execution.utils.logging import Logger
from antlr4.InputStream import InputStream
import py_trees


class TestExpression(unittest.TestCase):
    # pylint: disable=missing-function-docstring, protected-access, no-member, unused-variable
    """
    Unit test for expression parsing
    """

    def setUp(self) -> None:
        self.parser = OpenScenario2Parser(Logger('test', False))
        self.tree = py_trees.composites.Sequence(name="", memory=True)

    def parse(self, scenario_content):
        parsed_tree = self.parser.parse_input_stream(InputStream(scenario_content))
        return self.parser.create_internal_model(parsed_tree, self.tree, "test.osc", False)

    def test_add(self):
        scenario_content = """
type time is SI(s: 1)
unit s          of time is SI(s: 1, factor: 1)
unit ms         of time is SI(s: 1, factor: 0.001)

global test1: float = 2.0 + 1.1
global test2: time = 2.0s + 1.1s
global test3: time = 2.0s + 1ms
"""
        model = self.parse(scenario_content)
        self.assertAlmostEqual(model._ModelElement__children[3].get_resolved_value(), 3.1)
        self.assertAlmostEqual(model._ModelElement__children[4].get_resolved_value(), 3.1)
        self.assertAlmostEqual(model._ModelElement__children[5].get_resolved_value(), 2.001)

    def test_add_different_types(self):
        scenario_content = """
type time is SI(s: 1)
unit s          of time is SI(s: 1, factor: 1)

global test2: time = 2.0s + 1.1
"""
        self.assertRaises(ValueError, self.parse, scenario_content)

    def test_substract(self):
        scenario_content = """
type time is SI(s: 1)
unit s          of time is SI(s: 1, factor: 1)
unit ms         of time is SI(s: 1, factor: 0.001)

global test1: float = 2.0 - 1.1
global test2: time = 2.0s - 1.1s
global test3: time = 2.0s - 1ms
"""
        model = self.parse(scenario_content)
        self.assertAlmostEqual(model._ModelElement__children[3].get_resolved_value(), 0.9)
        self.assertAlmostEqual(model._ModelElement__children[4].get_resolved_value(), 0.9)
        self.assertAlmostEqual(model._ModelElement__children[5].get_resolved_value(), 1.999)

    def test_substract_different_types(self):
        scenario_content = """
type time is SI(s: 1)
unit s          of time is SI(s: 1, factor: 1)

global test2: time = 2.0s - 1.1
"""
        self.assertRaises(ValueError, self.parse, scenario_content)

    def test_multiply(self):
        scenario_content = """
type time is SI(s: 1)
unit ms         of time is SI(s: 1, factor: 0.001)

global test1: float = 2.0 * 1.1
global test2: time = 2.0ms * 1.1
"""
        model = self.parse(scenario_content)
        self.assertAlmostEqual(model._ModelElement__children[2].get_resolved_value(), 2.2)
        self.assertAlmostEqual(model._ModelElement__children[3].get_resolved_value(), 0.0022)

    def test_divide(self):
        scenario_content = """
type time is SI(s: 1)
unit ms         of time is SI(s: 1, factor: 0.001)

global test1: float = 5.0 / 2.0
global test2: time = 5.0ms / 2.0
"""
        model = self.parse(scenario_content)
        self.assertAlmostEqual(model._ModelElement__children[2].get_resolved_value(), 2.5)
        self.assertAlmostEqual(model._ModelElement__children[3].get_resolved_value(), 0.0025)

    def test_relation(self):
        scenario_content = """
type time is SI(s: 1)
unit ms         of time is SI(s: 1, factor: 0.001)

global test1: bool = 5.0 > 2.0
global test2: bool = 5 > 2
global test3: bool = 5 < 2
global test4: bool = 5 == 2
global test5: bool = 5 != 2
global test6: bool = 5 >= 2
global test7: bool = 5 <= 2
"""
        model = self.parse(scenario_content)
        self.assertEqual(model._ModelElement__children[2].get_resolved_value(), True)
        self.assertEqual(model._ModelElement__children[3].get_resolved_value(), True)
        self.assertEqual(model._ModelElement__children[4].get_resolved_value(), False)
        self.assertEqual(model._ModelElement__children[5].get_resolved_value(), False)
        self.assertEqual(model._ModelElement__children[6].get_resolved_value(), True)
        self.assertEqual(model._ModelElement__children[7].get_resolved_value(), True)
        self.assertEqual(model._ModelElement__children[8].get_resolved_value(), False)

    def test_negation(self):
        scenario_content = """
global test1: bool = not true
global test1: bool = not 5 > 2
global test1: bool = not false
"""
        model = self.parse(scenario_content)
        self.assertEqual(model._ModelElement__children[0].get_resolved_value(), False)
        self.assertEqual(model._ModelElement__children[1].get_resolved_value(), False)
        self.assertEqual(model._ModelElement__children[2].get_resolved_value(), True)

    def test_compound_expression(self):
        scenario_content = """
global test1: bool = 2 > 1 and 3 >= 2
global test1: bool = 2 > 1 or 3 < 2
"""
        model = self.parse(scenario_content)
        self.assertEqual(model._ModelElement__children[0].get_resolved_value(), True)
        self.assertEqual(model._ModelElement__children[1].get_resolved_value(), True)
