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
Test for Intermediate Scenario model Serialization and Deserialization
"""
import unittest

from scenario_execution.model.osc2_parser import OpenScenario2Parser
from scenario_execution.utils.logging import Logger
from scenario_execution.model.types import print_tree, serialize, deserialize
from .common import DebugLogger
from antlr4.InputStream import InputStream
import py_trees


class TestOSC2Parser(unittest.TestCase):
    # pylint: disable=missing-function-docstring, protected-access, no-member, unused-variable

    def setUp(self) -> None:
        self.parser = OpenScenario2Parser(Logger('test', False))
        self.tree = py_trees.composites.Sequence(name="", memory=True)

    def parse(self, scenario_content):
        parsed_tree = self.parser.parse_input_stream(InputStream(scenario_content))
        return self.parser.create_internal_model(parsed_tree, self.tree, "test.osc", False)

    def test_serialize(self):
        scenario_content = """
import osc.helpers

action log:
    msg: string

scenario test:
    do serial:
        log("foo")
"""
        model = self.parse(scenario_content)
        serialize_data = serialize(model)['CompilationUnit']['_children']
        self.assertGreater(len(serialize_data), 0)
        deserialized_model = deserialize(serialize_data)

        # validate model
        deserialize_model_logger = DebugLogger('out')
        input_model_logger = DebugLogger('in')
        print_tree(deserialized_model, deserialize_model_logger)
        print_tree(model, input_model_logger)
        self.assertListEqual(deserialize_model_logger.logs, input_model_logger.logs)

        self.assertGreater(len(deserialize_model_logger.logs), 0)
