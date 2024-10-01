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

import py_trees
import unittest
import tempfile
import os
from scenario_execution import ScenarioExecution
from scenario_execution.model.osc2_parser import OpenScenario2Parser
from scenario_execution.model.model_to_py_tree import create_py_tree
from scenario_execution.utils.logging import BaseLogger

from antlr4.InputStream import InputStream


class DebugLogger(BaseLogger):

    def __init__(self, name):
        super().__init__(name, False)
        self.logs = []

    def debug(self, msg: str) -> None:
        pass

    def warning(self, msg: str) -> None:
        pass

    def error(self, msg: str) -> None:
        pass

    def info(self, msg: str) -> None:
        self.logs.append(msg)


class TestCheckFileExists(unittest.TestCase):
    # pylint: disable=missing-function-docstring,missing-class-docstring

    def setUp(self) -> None:
        self.logger = DebugLogger("")
        self.parser = OpenScenario2Parser(self.logger)
        self.scenario_execution = ScenarioExecution(debug=False, log_model=False, live_tree=False,
                                                    scenario_file="test.osc", output_dir=None)
        self.tree = py_trees.composites.Sequence(name="", memory=True)
        self.tmp_file = tempfile.NamedTemporaryFile()

    def execute(self, scenario_content):
        parsed_tree = self.parser.parse_input_stream(InputStream(scenario_content))
        model = self.parser.create_internal_model(parsed_tree, self.tree, "test.osc", False)
        self.tree = create_py_tree(model, self.tree, self.parser.logger, False)
        self.scenario_execution.tree = self.tree
        self.scenario_execution.run()

    def test_paths(self):
        scenario_content = """
import osc.helpers
import osc.os
    
scenario test:
    timeout(1s)
    do serial:
        log(os.abspath('my_file'))
        log(os.basename('/tmp/bla'))
        log(os.dirname('/tmp/bla'))
"""
        self.execute(scenario_content)
        self.assertTrue(self.scenario_execution.process_results())
        self.assertEqual(self.logger.logs[0], os.path.join(os.getcwd(), "my_file"))
        self.assertEqual(self.logger.logs[1], "bla")
        self.assertEqual(self.logger.logs[2], "/tmp")
