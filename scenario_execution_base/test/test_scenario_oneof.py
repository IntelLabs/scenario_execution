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
Test oneof parsing
"""
import unittest
from datetime import datetime

from scenario_execution_base.scenario_execution import ScenarioExecution
from scenario_execution_base.model.osc2_parser import OpenScenario2Parser
from scenario_execution_base.model.model_to_py_tree import create_py_tree
from scenario_execution_base.utils.logging import Logger
from antlr4.InputStream import InputStream


class TestOSC2Parser(unittest.TestCase):
    # pylint: disable=missing-function-docstring, protected-access, no-member, unused-variable

    def setUp(self) -> None:
        self.parser = OpenScenario2Parser(Logger('test', False))
        self.scenario_execution = ScenarioExecution(debug=True, log_model=True, live_tree=True, scenario_file='test.osc', output_dir="")

    def test_oneof(self):
        scenario_content = """
type time is SI(s: 1)
unit s          of time is SI(s: 1, factor: 1)

scenario test:
    do serial:
        one_of:
            wait elapsed(120s)
            wait elapsed(5s)
        emit end        
"""
        parsed_tree = self.parser.parse_input_stream(InputStream(scenario_content))
        model = self.parser.create_internal_model(parsed_tree, "test.osc", True)
        scenarios = create_py_tree(model, self.parser.logger, False)
        self.scenario_execution.scenarios = scenarios

        start_time = datetime.now()
        self.scenario_execution.run()
        end_time = datetime.now()
        self.assertTrue(self.scenario_execution.process_results())

        delta = end_time - start_time
        self.assertLess(delta.total_seconds(), 15.)
