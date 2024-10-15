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
from scenario_execution.utils.logging import Logger
from antlr4.InputStream import InputStream
from datetime import datetime
from .common import DebugLogger


class TestOSC2Parser(unittest.TestCase):
    # pylint: disable=missing-function-docstring, protected-access, no-member, unused-variable
    """
    Unit test for osc2_parser
    """

    def setUp(self) -> None:
        self.logger = DebugLogger("")
        self.parser = OpenScenario2Parser(Logger('test', False))
        self.scenario_execution = ScenarioExecution(debug=True, log_model=True, live_tree=True, scenario_file='test.osc', output_dir="")
        self.tree = py_trees.composites.Sequence(name="", memory=True)

    def execute(self, scenario_content):
        self.logger.reset()
        parsed_tree = self.parser.parse_input_stream(InputStream(scenario_content))
        model = self.parser.create_internal_model(parsed_tree, self.tree, "test.osc", False)
        self.tree = create_py_tree(model, self.tree, self.logger, False)
        self.scenario_execution.tree = self.tree
        self.scenario_execution.run()

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
        self.execute(scenario_content)
        self.assertTrue(self.scenario_execution.process_results())

    def test_oneof(self):
        scenario_content = """
type time is SI(s: 1)
unit s          of time is SI(s: 1, factor: 1)

scenario test:
    do serial:
        one_of:
            wait elapsed(120s)
            wait elapsed(2s)
        emit end
"""
        parsed_tree = self.parser.parse_input_stream(InputStream(scenario_content))
        model = self.parser.create_internal_model(parsed_tree, self.tree, "test.osc", False)
        self.tree = create_py_tree(model, self.tree, self.parser.logger, False)
        self.scenario_execution.tree = self.tree

        start_time = datetime.now()
        self.scenario_execution.run()
        end_time = datetime.now()
        self.assertTrue(self.scenario_execution.process_results())

        delta = end_time - start_time
        self.assertLess(delta.total_seconds(), 15.)

    def test_serial(self):
        scenario_content = """
import osc.helpers

scenario test:
    do serial:
        log("A")
        serial:
            wait elapsed(0.5s)
            log("B")
        emit end
"""
        self.execute(scenario_content)
        self.assertTrue(self.scenario_execution.process_results())
        self.assertEqual(len(self.logger.logs_info), 2)
        self.assertEqual(self.logger.logs_info[0], "A")
        self.assertEqual(self.logger.logs_info[1], "B")

    def test_serial_no_memory(self):
        scenario_content = """
import osc.helpers

scenario test:
    do serial_no_memory:
        log("A")
        serial:
            wait elapsed(0.5s)
            log("B")
        emit end
"""
        self.execute(scenario_content)
        self.assertTrue(self.scenario_execution.process_results())
        self.assertGreater(len(self.logger.logs_info), 2)
        self.assertEqual(self.logger.logs_info[0], "A")
        self.assertEqual(self.logger.logs_info[1], "A")
        self.assertEqual(self.logger.logs_info[-1], "B")

    def test_selector(self):
        scenario_content = """
import osc.helpers

scenario test:
    do serial:
        repeat(2)
        selector:
            log("A")
            log("B")
"""
        self.execute(scenario_content)
        self.assertTrue(self.scenario_execution.process_results())
        self.assertEqual(len(self.logger.logs_info), 2)
        self.assertEqual(self.logger.logs_info[0], "A")
        self.assertEqual(self.logger.logs_info[1], "A")
        # self.assertEqual(self.logger.logs_info[-1], "B")

    def test_selector_no_memory(self):
        scenario_content = """
import osc.helpers

scenario test:
    do serial:
        repeat(2)
        selector_no_memory:
            run_process("false")
            log("B")
"""
        self.execute(scenario_content)
        self.assertTrue(self.scenario_execution.process_results())
        self.assertEqual(len(self.logger.logs_info), 2)
        self.assertEqual(self.logger.logs_info[0], "B")
        self.assertEqual(self.logger.logs_info[1], "B")

    def test_selector_no_memory_second_false(self):
        scenario_content = """
import osc.helpers

scenario test:
    do serial:
        repeat(2)
        selector_no_memory:
            log("A")
            run_process("false")
"""
        self.execute(scenario_content)
        self.assertTrue(self.scenario_execution.process_results())
        self.assertEqual(len(self.logger.logs_info), 2)
        self.assertEqual(self.logger.logs_info[0], "A")
        self.assertEqual(self.logger.logs_info[1], "A")

    def test_selector_no_memory_fail(self):
        scenario_content = """
import osc.helpers

scenario test:
    do serial:
        repeat(2)
        selector_no_memory:
            p1: run_process("false")
            p2: run_process("false")
"""
        self.execute(scenario_content)
        self.assertFalse(self.scenario_execution.process_results())
