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
import py_trees
from scenario_execution.scenario_execution_base import ScenarioExecution
from scenario_execution.model.osc2_parser import OpenScenario2Parser
from scenario_execution.model.model_to_py_tree import create_py_tree
from .common import DebugLogger
from antlr4.InputStream import InputStream


class TestOSC2Parser(unittest.TestCase):
    # pylint: disable=missing-function-docstring, protected-access, no-member, unused-variable

    def setUp(self) -> None:
        self.logger = DebugLogger("")
        self.parser = OpenScenario2Parser(self.logger)
        self.scenario_execution = ScenarioExecution(debug=False, log_model=False, live_tree=True,
                                                    scenario_file='test.osc', output_dir="", logger=self.logger)
        self.tree = py_trees.composites.Sequence(name="", memory=True)

    def execute(self, scenario_content):
        parsed_tree = self.parser.parse_input_stream(InputStream(scenario_content))
        model = self.parser.create_internal_model(parsed_tree, self.tree, "test.osc", False)
        self.tree = create_py_tree(model, self.tree, self.parser.logger, False)
        self.scenario_execution.tree = self.tree
        self.scenario_execution.run()

    def test_repeat_root(self):
        scenario_content = """
import osc.helpers

scenario test:
    do serial:
        repeat(2)
        log("TestA")
        log("TestB")
"""
        parsed_tree = self.parser.parse_input_stream(InputStream(scenario_content))
        model = self.parser.create_internal_model(parsed_tree, self.tree, "test.osc", False)
        create_py_tree(model, self.tree, self.parser.logger, False)
        self.scenario_execution.tree = self.tree
        self.scenario_execution.run()
        self.assertTrue(self.scenario_execution.process_results())
        self.assertEqual([
            "Executing scenario 'test'",
            "TestA", "TestB", "TestA", "TestB",
            "Scenario 'test' succeeded."], self.logger.logs_info)

    def test_repeat_sub_tree(self):
        scenario_content = """
import osc.helpers

scenario test:
    do serial:
        repeat(2)
        log("foo")
        serial:
            repeat(2)
            log("TestA")
            log("TestB")
        log("bar")
"""
        self.execute(scenario_content)
        self.assertTrue(self.scenario_execution.process_results())
        self.assertEqual([
            "Executing scenario 'test'",
            "foo",
            "TestA",
            "TestB",
            "TestA",
            "TestB",
            "bar",
            "foo",
            "TestA",
            "TestB",
            "TestA",
            "TestB",
            "bar",
            "Scenario 'test' succeeded."], self.logger.logs_info)

    def test_unknown_modifier(self):
        scenario_content = """
import osc.helpers

modifier not_supported

scenario test:
    do serial:
        not_supported()
"""
        parsed_tree = self.parser.parse_input_stream(InputStream(scenario_content))
        model = self.parser.create_internal_model(parsed_tree, self.tree, "test.osc", False)
        self.assertRaises(ValueError, create_py_tree, model, self.tree, self.parser.logger, False)

    def test_inverter_1(self):
        scenario_content = """
import osc.helpers

scenario test:
    do serial:
        inverter()
        run_process('false')
"""
        self.execute(scenario_content)
        self.assertTrue(self.scenario_execution.process_results())

    def test_inverter_2(self):
        scenario_content = """
import osc.helpers

scenario test:
    do serial:
        inverter()
        run_process('true')
"""
        self.execute(scenario_content)
        self.assertFalse(self.scenario_execution.process_results())

    def test_modifier_second(self):
        scenario_content = """
import osc.helpers

scenario test:
    do serial:
        run_process('false')
        inverter()
"""
        self.execute(scenario_content)
        self.assertTrue(self.scenario_execution.process_results())

    def test_timeout(self):
        scenario_content = """
import osc.helpers

scenario test:
    do serial:
        timeout(3s)
        wait elapsed(10s)
        emit end
"""
        self.execute(scenario_content)
        self.assertFalse(self.scenario_execution.process_results())

    def test_no_timeout(self):
        scenario_content = """
import osc.helpers

scenario test:
    do serial:
        timeout(10s)
        wait elapsed(3s)
        emit end
"""
        self.execute(scenario_content)
        self.assertTrue(self.scenario_execution.process_results())

    def test_retry(self):
        scenario_content = """
import osc.helpers

scenario test:
    do serial:
        retry(3)
        log("RETRY")
        run_process('false')
"""
        self.execute(scenario_content)
        self.assertFalse(self.scenario_execution.process_results())
        self.assertEqual(3, len([i for i in self.logger.logs_info if i == "RETRY"]))

    def test_retry_succeeds_in_first(self):
        scenario_content = """
import osc.helpers

scenario test:
    do serial:
        retry(3)
        log("RETRY")
        run_process('true')
"""
        self.execute(scenario_content)
        self.assertTrue(self.scenario_execution.process_results())

        self.assertEqual(1, len([i for i in self.logger.logs_info if i == "RETRY"]))

    def test_multi_inverter(self):
        scenario_content = """
import osc.helpers

scenario test:
    do serial:
        inverter() # -> success
        inverter() # -> failure
        inverter() # -> success
        run_process('false')
"""
        self.execute(scenario_content)
        self.assertTrue(self.scenario_execution.process_results())

    def test_multi_1(self):
        scenario_content = """
import osc.helpers

scenario test:
    do serial:
        retry(3)
        timeout(3s)
        inverter()
        wait elapsed(0.5s)
        log("RETRY")
        run_process('true')
"""
        self.execute(scenario_content)
        self.assertFalse(self.scenario_execution.process_results())
        self.assertEqual(3, len([i for i in self.logger.logs_info if i == "RETRY"]))

    def test_multi_order1(self):
        scenario_content = """
import osc.helpers

scenario test:
    do serial:
        retry(3)
        timeout(2s) # timeout for single run (does not trigger)
        inverter()
        log("RETRY")
        wait elapsed(1s)
        run_process('true')
"""
        self.execute(scenario_content)
        self.assertFalse(self.scenario_execution.process_results())
        self.assertEqual(3, len([i for i in self.logger.logs_info if i == "RETRY"]))

    def test_multi_order2(self):
        scenario_content = """
import osc.helpers

scenario test:
    do serial:
        timeout(2.5s) # timeout for all runs (succeeds)
        retry(3)
        inverter()
        wait elapsed(1s)
        log("RETRY")
        run_process('true')
"""
        self.execute(scenario_content)
        self.assertFalse(self.scenario_execution.process_results())
        self.assertEqual(2, len([i for i in self.logger.logs_info if i == "RETRY"]))

    def test_multi_order3(self):
        scenario_content = """
import osc.helpers

scenario test:
    do serial:
        timeout(4s) # timeout for all runs (succeeds)
        retry(3)
        inverter()
        log("RETRY")
        wait elapsed(0.5s)
        run_process('true')
"""
        self.execute(scenario_content)
        self.assertFalse(self.scenario_execution.process_results())
        self.assertEqual(3, len([i for i in self.logger.logs_info if i == "RETRY"]))

    def test_modifier_in_action(self):
        scenario_content = """
import osc.helpers

scenario test:
    do serial:
        run_process('false') with:
            inverter()
"""
        self.execute(scenario_content)
        self.assertTrue(self.scenario_execution.process_results())

    def test_modifier_in_action_and_modifier_in_serial(self):
        scenario_content = """
import osc.helpers

scenario test:
    do serial:
        repeat(2)
        log("RETRY")
        run_process('false') with:
            inverter()
"""
        self.execute(scenario_content)
        self.assertTrue(self.scenario_execution.process_results())
        self.assertEqual(2, len([i for i in self.logger.logs_info if i == "RETRY"]))

    def test_two_modifier_in_action_and_modifier_in_serial(self):
        scenario_content = """
import osc.helpers

scenario test:
    do serial:
        repeat(2)
        log('RETRY') with:
            repeat(2)
        run_process('false') with:
            inverter()
"""
        self.execute(scenario_content)
        self.assertTrue(self.scenario_execution.process_results())
        self.assertEqual(4, len([i for i in self.logger.logs_info if i == "RETRY"]))

    def test_modifier_in_action_second_parameter(self):
        scenario_content = """
import osc.helpers

scenario test:
    do serial:
        run_process() with:
            keep(it.command == "false")
            inverter()
"""
        self.execute(scenario_content)
        self.assertTrue(self.scenario_execution.process_results())

    def test_modifier_in_scenario(self):
        scenario_content = """
import osc.helpers

scenario test:
    timeout(2s)
    do serial:
        log("TEST")
"""
        self.execute(scenario_content)
        self.assertTrue(self.scenario_execution.process_results())

    def test_modifier_in_scenario_fails(self):
        scenario_content = """
import osc.helpers

scenario test:
    timeout(2s)
    do serial:
        log("TEST")
        wait elapsed(3s)
"""
        self.execute(scenario_content)
        self.assertFalse(self.scenario_execution.process_results())

    def test_modifier_in_scenario_last_fails(self):
        scenario_content = """
import osc.helpers

scenario test:
    do serial:
        log("TEST")
        wait elapsed(3s)
    timeout(2s)
"""
        self.execute(scenario_content)
        self.assertFalse(self.scenario_execution.process_results())
