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
from scenario_execution import ScenarioExecution
from scenario_execution.model.osc2_parser import OpenScenario2Parser
from scenario_execution.model.model_to_py_tree import create_py_tree
from scenario_execution.utils.logging import Logger

from antlr4.InputStream import InputStream


class TestDockerCopy(unittest.TestCase):
    # pylint: disable=missing-function-docstring,missing-class-docstring

    def setUp(self) -> None:
        self.parser = OpenScenario2Parser(Logger('test', False))
        self.tmp_dir = tempfile.TemporaryDirectory()
        self.scenario_execution = ScenarioExecution(debug=False, log_model=False, live_tree=False,
                                                    scenario_file="test.osc", output_dir=self.tmp_dir.name)
        self.tree = py_trees.composites.Sequence(name="", memory=True)

    def tearDown(self):
        self.tmp_dir.cleanup()


    def parse(self, scenario_content):
        parsed_tree = self.parser.parse_input_stream(InputStream(scenario_content))
        model = self.parser.create_internal_model(parsed_tree, self.tree, "test.osc", False)
        self.tree = create_py_tree(model, self.tree, self.parser.logger, False)
        self.scenario_execution.tree = self.tree
        self.scenario_execution.run()

    def test_success(self):
        self.parse("""
import osc.docker
import osc.helpers
import osc.os

scenario test_success:
    timeout(10)
    do parallel:
        docker_run(image: 'ubuntu', command: 'sleep 5', detach: true, container_name: 'sleeping_beauty', remove: true)
        serial: 
            docker_exec(container: 'sleeping_beauty', command: 'mkdir -p /tmp/test_dir/')
            docker_exec(container: 'sleeping_beauty', command: 'touch /tmp/test_dir/test.txt')
            docker_exec(container: 'sleeping_beauty', command: 'touch /tmp/test_dir/test_1.txt')
            docker_copy(container: 'sleeping_beauty', file_path:  '/tmp/test_dir/')
            check_file_exists(file_name: '""" + self.tmp_dir.name + '/test_dir/test.txt' + """')
            check_file_exists(file_name: '""" + self.tmp_dir.name + '/test_dir/test_1.txt' + """')
            emit end
""")
        self.assertTrue(self.scenario_execution.process_results())

    def test_failure_missing_file(self):
        self.parse("""
import osc.docker
import osc.helpers

scenario test_success:
    timeout(10s)
    do parallel:
        docker_run(image: 'ubuntu', command: 'sleep 5', detach: true, container_name: 'sleeping_beauty1', remove: true)
        serial: 
            docker_copy(container: 'sleeping_beauty1', file_path:  '/tmp/test_dir/')
""")
        self.assertFalse(self.scenario_execution.process_results())
        