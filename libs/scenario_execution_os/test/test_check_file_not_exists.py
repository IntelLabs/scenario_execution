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
import tempfile
import py_trees
from scenario_execution import ScenarioExecution
from scenario_execution.model.osc2_parser import OpenScenario2Parser
from scenario_execution.model.model_to_py_tree import create_py_tree
from scenario_execution.utils.logging import Logger

from antlr4.InputStream import InputStream


class TestCheckData(unittest.TestCase):
    # pylint: disable=missing-function-docstring,missing-class-docstring

    def setUp(self) -> None:
        self.parser = OpenScenario2Parser(Logger('test', False))
        self.scenario_execution = ScenarioExecution(debug=False, log_model=False, live_tree=False,
                                                    scenario_file="test.osc", output_dir=None)
        self.tree = py_trees.composites.Sequence()
        self.tmp_file = tempfile.NamedTemporaryFile()

    def parse(self, scenario_content):
        parsed_tree = self.parser.parse_input_stream(InputStream(scenario_content))
        model = self.parser.create_internal_model(parsed_tree, self.tree, "test.osc", False)
        create_py_tree(model, self.tree, self.parser.logger, log_tree=False)
        self.scenario_execution.tree = self.tree
        self.scenario_execution.run()

    def test_success(self):
        self.parse("""
import osc.os

scenario test:
    do parallel:
        serial:
            check_file_not_exists('UNKNOWN_FILE')
            emit end
        time_out: serial:
            wait elapsed(1s)
            emit fail
""")
        self.assertTrue(self.scenario_execution.process_results())

    def test_fail(self):
        self.parse("""
import osc.os

scenario test:
    do parallel:
        serial:
            check_file_not_exists('""" + self.tmp_file.name + """')
            emit end
        time_out: serial:
            wait elapsed(1s)
            emit fail
""")
        self.assertFalse(self.scenario_execution.process_results())
