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
Test event parsing
"""
import unittest
from scenario_execution.model.osc2_parser import OpenScenario2Parser
from scenario_execution.utils.logging import Logger
from antlr4.InputStream import InputStream


class TestScenarioExectionSuccess(unittest.TestCase):
    # pylint: disable=missing-function-docstring, protected-access, no-member, unused-variable

    def setUp(self) -> None:
        self.parser = OpenScenario2Parser(Logger('test', False))

    def test_unknown_event(self):
        scenario_content = """
scenario test:
    do serial:
        emit UNKNOWN
"""
        parsed_tree = self.parser.parse_input_stream(InputStream(scenario_content))
        self.assertRaises(ValueError, self.parser.create_internal_model, parsed_tree, "test.osc", False)

    def test_event_success(self):
        scenario_content = """
scenario test:
    event test1
    do serial:
        emit test1
"""
        parsed_tree = self.parser.parse_input_stream(InputStream(scenario_content))
        model = self.parser.create_internal_model(parsed_tree, "test.osc", False)

    def test_event_end(self):
        scenario_content = """
scenario test:
    do serial:
        emit end
"""
        parsed_tree = self.parser.parse_input_stream(InputStream(scenario_content))
        model = self.parser.create_internal_model(parsed_tree, "test.osc", False)
