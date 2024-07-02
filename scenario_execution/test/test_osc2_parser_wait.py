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
Test wait parsing
"""
import unittest
import py_trees
from scenario_execution.model.osc2_parser import OpenScenario2Parser
from scenario_execution.model.model_to_py_tree import create_py_tree
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

    def test_wait_success(self):
        scenario_content = """
type time is SI(s: 1)
unit s          of time is SI(s: 1, factor: 1)

scenario test:
    do serial:
        wait elapsed(1s)
"""
        model = self.parse(scenario_content)
        create_py_tree(model, self.tree, self.parser.logger, False)

    def test_wait_invalid(self):
        scenario_content = """
type time is SI(s: 1)
unit s          of time is SI(s: 1, factor: 1)

scenario test:
    do serial:
        wait(1s)
"""
        model = self.parse(scenario_content)
        self.assertRaises(ValueError, create_py_tree, model, self.tree, self.parser.logger, False)

    def test_wait_invalid_literal(self):
        scenario_content = """
scenario test:
    do serial:
        wait(1)
"""
        model = self.parse(scenario_content)
        self.assertRaises(ValueError, create_py_tree, model, self.tree, self.parser.logger, False)

    def test_wait_invalid_literal2(self):
        scenario_content = """
scenario test:
    do serial:
        wait elapsed(1)
"""
        model = self.parse(scenario_content)
        self.assertRaises(ValueError, create_py_tree, model, self.tree, self.parser.logger, False)
