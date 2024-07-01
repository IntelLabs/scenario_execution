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

import os
import unittest
import rclpy
import py_trees
import tempfile
import threading
from scenario_execution_ros import ROSScenarioExecution
from scenario_execution.model.osc2_parser import OpenScenario2Parser
from scenario_execution.model.model_to_py_tree import create_py_tree
from scenario_execution.utils.logging import Logger
from antlr4.InputStream import InputStream

os.environ["PYTHONUNBUFFERED"] = '1'


class TestScenarioExectionSuccess(unittest.TestCase):
    # pylint: disable=missing-function-docstring

    def setUp(self):
        rclpy.init()
        self.request_received = None
        self.node = rclpy.create_node('test_node_action')

        self.executor = rclpy.executors.MultiThreadedExecutor()
        self.executor.add_node(self.node)
        self.executor_thread = threading.Thread(target=self.executor.spin)
        self.executor_thread.start()

        self.parser = OpenScenario2Parser(Logger('test', False))
        self.scenario_execution_ros = ROSScenarioExecution()
        self.tmp_dir = tempfile.TemporaryDirectory()
        self.tree = py_trees.composites.Sequence()

    def tearDown(self):
        self.node.destroy_node()
        rclpy.try_shutdown()
        self.executor_thread.join()
        self.tmp_dir.cleanup()

    def execute(self, scenario_content):
        parsed_tree = self.parser.parse_input_stream(InputStream(scenario_content))
        model = self.parser.create_internal_model(parsed_tree, self.tree, "test.osc", False)
        create_py_tree(model, self.tree, self.parser.logger, False)
        self.scenario_execution_ros.tree = self.tree
        self.scenario_execution_ros.run()

    def test_success(self):
        scenario_content = """
import osc.ros
import osc.os

scenario test:
    do parallel:
        serial:
            ros_launch('test_scenario_execution_ros', 'test_launch.py', [
                ros_argument(key: 'test_param', value: '""" + self.tmp_dir.name + """'),
                ros_argument(key: 'test_path', value: '""" + self.tmp_dir.name + """')
            ])
            check_file_exists(file_name: '""" + self.tmp_dir.name + '/test_started' + """')
            check_file_exists(file_name: '""" + self.tmp_dir.name + '/test_success' + """')
            check_file_not_exists(file_name: '""" + self.tmp_dir.name + '/test_aborted' + """')
            emit end
        time_out: serial:
            wait elapsed(10s)
            emit fail
"""
        self.execute(scenario_content)
        self.assertTrue(self.scenario_execution_ros.process_results())

    def test_success_not_wait_for_shutdown(self):
        scenario_content = """
import osc.ros
import osc.os

scenario test:
    do parallel:
        serial:
            ros_launch('test_scenario_execution_ros', 'test_launch.py', [
                    ros_argument(key: 'test_param', value: '""" + self.tmp_dir.name + """'),
                    ros_argument(key: 'test_path', value: '""" + self.tmp_dir.name + """')
                ],
                wait_for_shutdown: false)
            wait elapsed(4s)
            check_file_exists(file_name: '""" + self.tmp_dir.name + '/test_started' + """')
            check_file_not_exists(file_name: '""" + self.tmp_dir.name + '/test_success' + """')
            check_file_not_exists(file_name: '""" + self.tmp_dir.name + '/test_aborted' + """')
            wait elapsed(10s)
            check_file_exists(file_name: '""" + self.tmp_dir.name + '/test_started' + """')
            check_file_exists(file_name: '""" + self.tmp_dir.name + '/test_success' + """')
            check_file_not_exists(file_name: '""" + self.tmp_dir.name + '/test_aborted' + """')
            emit end
        time_out: serial:
            wait elapsed(20s)
            emit fail
"""
        self.execute(scenario_content)
        self.assertTrue(self.scenario_execution_ros.process_results())

    def test_success_not_wait_for_shutdown_terminate(self):
        scenario_content = """
import osc.ros
import osc.os

scenario test:
    do serial:
        ros_launch('test_scenario_execution_ros', 'test_launch.py', [ 
                ros_argument(key: 'test_param', value: '""" + self.tmp_dir.name + """'),
                ros_argument(key: 'test_path', value: '""" + self.tmp_dir.name + """'),
                ros_argument(key: 'timeout', value: '15')
            ],
            wait_for_shutdown: false,
            shutdown_timeout: 5s)
        wait elapsed(2s)
        emit end
"""
        self.execute(scenario_content)
        self.assertTrue(os.path.isfile(self.tmp_dir.name + '/test_started'))
        self.assertFalse(os.path.isfile(self.tmp_dir.name + '/test_success'))
        self.assertTrue(os.path.isfile(self.tmp_dir.name + '/test_aborted'))
        self.assertTrue(self.scenario_execution_ros.process_results())
