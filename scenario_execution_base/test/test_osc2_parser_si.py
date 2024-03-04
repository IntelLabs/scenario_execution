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

from scenario_execution_base.model.osc2_parser import OpenScenario2Parser
from scenario_execution_base.utils.logging import Logger
from antlr4.InputStream import InputStream


class TestOSC2Parser(unittest.TestCase):
    # pylint: disable=missing-function-docstring, protected-access, no-member, unused-variable

    def setUp(self) -> None:
        self.parser = OpenScenario2Parser(Logger('test'))

    def test_si_invalid_type(self):
        scenario_content = """
type length is SI(m: 1)
unit cm         of length is SI(m: 1, factor: 0.01)

global val1: string = 3.2cm
"""
        parsed_tree, errors = self.parser.parse_input_stream(InputStream(scenario_content))
        self.assertEqual(errors, 0)
        model = self.parser.create_internal_model(parsed_tree, "test.osc", True)
        self.assertIsNone(model)

    def test_si(self):
        scenario_content = """
type length is SI(m: 1)
unit cm         of length is SI(m: 1, factor: 0.01)

global val1: length = 3.2cm
"""
        parsed_tree, errors = self.parser.parse_input_stream(InputStream(scenario_content))
        self.assertEqual(errors, 0)
        model = self.parser.create_internal_model(parsed_tree, "test.osc", True)
        self.assertIsNotNone(model)

        param = model._ModelElement__children[2]
        self.assertEqual(param.get_resolved_value(), 0.032)

    def test_si_naming(self):
        scenario_content = """
type length is SI(m: 1)
unit m          of length is SI(m: 1, factor: 1)
global val1: length = 3.2m
"""
        parsed_tree, errors = self.parser.parse_input_stream(InputStream(scenario_content))
        self.assertEqual(errors, 0)
        model = self.parser.create_internal_model(parsed_tree, "test.osc", True)
        self.assertIsNotNone(model)
        param = model._ModelElement__children[2]
        self.assertEqual(param.get_resolved_value(), 3.2)

    def test_si_unknown_type(self):
        scenario_content = """
type length is SI(m: 1)
unit m          of UNKNOWN is SI(m: 1, factor: 1)
"""
        parsed_tree, errors = self.parser.parse_input_stream(InputStream(scenario_content))
        self.assertEqual(errors, 0)
        model = self.parser.create_internal_model(parsed_tree, "test.osc")
        self.assertIsNone(model)
