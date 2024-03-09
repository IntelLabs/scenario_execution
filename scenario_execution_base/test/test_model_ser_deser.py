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

from scenario_execution_base.model.osc2_parser import OpenScenario2Parser
from scenario_execution_base.model.model_file_loader import ModelFileLoader
from scenario_execution_base.utils.logging import BaseLogger, Logger
from scenario_execution_base.model.types import print_tree, serialize, deserialize
import os
from antlr4.InputStream import InputStream


class DebugLogger(BaseLogger):

    def __init__(self, name):
        super().__init__(name)
        self.logs = []

    def info(self, msg: str) -> None:
        self.logs.append(msg)

    def debug(self, msg: str) -> None:
        self.logs.append(msg)

    def warning(self, msg: str) -> None:
        self.logs.append(msg)

    def error(self, msg: str) -> None:
        self.logs.append(msg)


class TestOSC2Parser(unittest.TestCase):
    # pylint: disable=missing-function-docstring, protected-access, no-member, unused-variable

    def setUp(self) -> None:
        self.parser = OpenScenario2Parser(Logger('test'))

    def test_serialize(self):
        scenario_content = """
import osc.helpers

action log:
    msg: string

scenario test:
    do serial:
        log("foo")
        emit end
"""
        parsed_tree, errors = self.parser.parse_input_stream(InputStream(scenario_content))
        self.assertEqual(errors, 0)
        model = self.parser.load_internal_model(parsed_tree, "test.osc", False, False)
        self.assertIsNotNone(model)
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
