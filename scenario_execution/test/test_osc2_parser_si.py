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
Test si parsing
"""
import unittest
import py_trees
from scenario_execution.model.osc2_parser import OpenScenario2Parser
from scenario_execution.utils.logging import Logger
from antlr4.InputStream import InputStream


class TestOSC2Parser(unittest.TestCase):
    # pylint: disable=missing-function-docstring, protected-access, no-member, unused-variable

    def setUp(self) -> None:
        self.parser = OpenScenario2Parser(Logger('test', False))
        self.tree = py_trees.composites.Sequence()

    def parse(self, scenario_content):
        parsed_tree = self.parser.parse_input_stream(InputStream(scenario_content))
        return self.parser.create_internal_model(parsed_tree, self.tree, "test.osc", False)

    def test_si_invalid_type(self):
        scenario_content = """
type length is SI(m: 1)
unit cm         of length is SI(m: 1, factor: 0.01)

global val1: string = 3.2cm
"""
        parsed_tree = self.parser.parse_input_stream(InputStream(scenario_content))
        self.assertRaises(ValueError, self.parser.create_internal_model, parsed_tree, self.tree, "test.osc")

    def test_si(self):
        scenario_content = """
type length is SI(m: 1)
unit cm         of length is SI(m: 1, factor: 0.01)

global val1: length = 3.2cm
"""
        model = self.parse(scenario_content)
        param = model._ModelElement__children[2]
        self.assertEqual(param.get_resolved_value(), 0.032)

    def test_si_naming(self):
        scenario_content = """
type length is SI(m: 1)
unit m          of length is SI(m: 1, factor: 1)
global val1: length = 3.2m
"""
        model = self.parse(scenario_content)
        param = model._ModelElement__children[2]
        self.assertEqual(param.get_resolved_value(), 3.2)

    def test_si_unknown_type(self):
        scenario_content = """
type length is SI(m: 1)
unit m          of UNKNOWN is SI(m: 1, factor: 1)
"""
        parsed_tree = self.parser.parse_input_stream(InputStream(scenario_content))
        self.assertRaises(ValueError, self.parser.create_internal_model, parsed_tree, self.tree, "test.osc")
