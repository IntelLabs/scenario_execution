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
import os
from scenario_execution_base.scenario_execution import ScenarioExecution


class TestOSC2Parser(unittest.TestCase):
    # pylint: disable=missing-function-docstring, protected-access, no-member, unused-variable

    def test_scenario_file_not_existing(self):
        scenario_execution = ScenarioExecution(debug=False,
                                               log_model=False,
                                               live_tree=False,
                                               scenario_file='UNKNOWN_FILE.sce',
                                               output_dir="")
        result = scenario_execution.parse()
        self.assertFalse(result)

    def test_scenario_file_wrong_extension(self):
        scenario_execution = ScenarioExecution(debug=False,
                                               log_model=False,
                                               live_tree=False,
                                               scenario_file='UNKNOWN_FILE',
                                               output_dir="")
        result = scenario_execution.parse()
        self.assertFalse(result)

    def test_no_scenario(self):
        scenario_execution = ScenarioExecution(debug=False,
                                               log_model=False,
                                               live_tree=False,
                                               scenario_file=os.path.join(os.path.dirname(__file__), 'scenarios', 'no_scenario.osc'),
                                               output_dir="")
        result = scenario_execution.parse()
        self.assertFalse(result)

    def test_scenario(self):
        scenario_execution = ScenarioExecution(debug=False,
                                               log_model=False,
                                               live_tree=False,
                                               scenario_file=os.path.join(os.path.dirname(__file__), 'scenarios', 'test_scenario.osc'),
                                               output_dir="")
        result = scenario_execution.parse()
        self.assertTrue(result)

    def test_two_scenarios(self):
        scenario_execution = ScenarioExecution(debug=False,
                                               log_model=False,
                                               live_tree=False,
                                               scenario_file=os.path.join(os.path.dirname(__file__), 'scenarios', 'two_scenarios.osc'),
                                               output_dir="")
        result = scenario_execution.parse()
        self.assertFalse(result)
