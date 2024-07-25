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
import rclpy
import py_trees
from scenario_execution_ros import ROSScenarioExecution
from scenario_execution.model.osc2_parser import OpenScenario2Parser
from scenario_execution.model.model_to_py_tree import create_py_tree
from scenario_execution.utils.logging import Logger
from antlr4.InputStream import InputStream


class TestRosPublishReceive(unittest.TestCase):
    # pylint: disable=missing-function-docstring

    def setUp(self) -> None:
        rclpy.init()
        self.parser = OpenScenario2Parser(Logger('test', False))
        self.scenario_execution_ros = ROSScenarioExecution()
        self.tree = py_trees.composites.Sequence(name="", memory=True)

    def tearDown(self):
        rclpy.try_shutdown()

    def test_success(self):
        scenario_content = """
import osc.helpers
import osc.ros

scenario test_ros_topic_publish:
    timeout(10s)
    do parallel:
        test: serial:
            wait elapsed(1s)
            topic_publish() with:
                keep(it.topic_name == '/bla')
                keep(it.topic_type == 'std_msgs.msg.Bool')
                keep(it.value == '{\\\"data\\\": True}')
        receive: serial:
            wait_for_data() with:
                keep(it.topic_name == '/bla')
                keep(it.topic_type == 'std_msgs.msg.Bool')
            emit end
"""
        parsed_tree = self.parser.parse_input_stream(InputStream(scenario_content))
        model = self.parser.create_internal_model(parsed_tree, self.tree, "test.osc", False)
        self.tree = create_py_tree(model, self.tree, self.parser.logger, False)
        self.scenario_execution_ros.tree = self.tree
        self.scenario_execution_ros.run()
        self.assertTrue(self.scenario_execution_ros.process_results())
