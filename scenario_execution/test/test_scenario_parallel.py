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
Test parallel parsing
"""
import unittest

from scenario_execution.scenario_execution_base import ScenarioExecution
from scenario_execution.model.osc2_parser import OpenScenario2Parser
from scenario_execution.model.model_to_py_tree import create_py_tree
from scenario_execution.utils.logging import Logger
from antlr4.InputStream import InputStream


class TestOSC2Parser(unittest.TestCase):
    # pylint: disable=missing-function-docstring, protected-access, no-member, unused-variable
    """
    Unit test for osc2_parser
    """

    def setUp(self) -> None:
        self.parser = OpenScenario2Parser(Logger('test', False))
        self.scenario_execution = ScenarioExecution(debug=True, log_model=True, live_tree=True, scenario_file='test.osc', output_dir="")

    def test_parallel(self):
        scenario_content = """
type time is SI(s: 1)
unit s          of time is SI(s: 1, factor: 1)

scenario test:
    do serial:
        parallel:
            serial:
                wait elapsed(5s)
                emit end
            serial:
                wait elapsed(1s)
        wait elapsed(1s)
        emit fail
"""
        parsed_tree = self.parser.parse_input_stream(InputStream(scenario_content))
        model = self.parser.create_internal_model(parsed_tree, "test.osc", False)
        scenarios = create_py_tree(model, self.parser.logger, False)
        self.scenario_execution.scenarios = scenarios
        self.scenario_execution.run()
        self.assertTrue(self.scenario_execution.process_results())
