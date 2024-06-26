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
from scenario_execution import ScenarioExecution
from scenario_execution.model.osc2_parser import OpenScenario2Parser
from scenario_execution.model.model_to_py_tree import create_py_tree
from scenario_execution.utils.logging import Logger

from antlr4.InputStream import InputStream


class TestCheckData(unittest.TestCase):
    # pylint: disable=missing-function-docstring,missing-class-docstring

    def setUp(self) -> None:
        self.parser = OpenScenario2Parser(Logger('test', False))
        self.scenario_execution = ScenarioExecution(debug=False, log_model=False, live_tree=False, scenario_file="test.osc", output_dir=None)
        self.tmp_file = tempfile.NamedTemporaryFile()
        print(self.tmp_file.name)

    def test_success(self):
        scenario_content = """
import osc.os

scenario test:
    do parallel:
        serial:
            check_file_exists('""" + self.tmp_file.name + """')
            emit end
        time_out: serial:
            wait elapsed(1s)
            emit fail
"""

        parsed_tree = self.parser.parse_input_stream(InputStream(scenario_content))
        model = self.parser.create_internal_model(parsed_tree, "test.osc", False)
        scenarios = create_py_tree(model, self.parser.logger, False)
        self.scenario_execution.scenarios = scenarios
        self.scenario_execution.run()
        self.assertTrue(self.scenario_execution.process_results())

    def test_fail(self):
        scenario_content = """
import osc.os

scenario test:
    do parallel:
        serial:
            check_file_exists('UNKNOWN_FILE')
            emit end
        time_out: serial:
            wait elapsed(1s)
            emit fail
"""

        parsed_tree = self.parser.parse_input_stream(InputStream(scenario_content))
        model = self.parser.create_internal_model(parsed_tree, "test.osc", False)
        scenarios = create_py_tree(model, self.parser.logger, False)
        self.scenario_execution.scenarios = scenarios
        self.scenario_execution.run()
        self.assertFalse(self.scenario_execution.process_results())
