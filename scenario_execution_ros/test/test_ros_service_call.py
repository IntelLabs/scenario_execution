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
import threading
from scenario_execution_ros import ROSScenarioExecution
from scenario_execution.model.osc2_parser import OpenScenario2Parser
from scenario_execution.utils.logging import Logger
from scenario_execution.model.model_to_py_tree import create_py_tree
from ament_index_python.packages import get_package_share_directory
from antlr4.InputStream import InputStream
import py_trees
from std_srvs.srv import SetBool

os.environ["PYTHONUNBUFFERED"] = '1'


class TestRosServiceCall(unittest.TestCase):
    # pylint: disable=missing-function-docstring

    def setUp(self):
        rclpy.init()
        self.request_received = None
        self.node = rclpy.create_node('test_node')

        self.executor = rclpy.executors.MultiThreadedExecutor()
        self.executor.add_node(self.node)
        self.executor_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.executor_thread.start()

        self.scenario_dir = get_package_share_directory('scenario_execution_ros')

        self.srv = self.node.create_service(SetBool, "/bla", self.service_callback)
        self.parser = OpenScenario2Parser(Logger('test', False))
        self.scenario_execution_ros = ROSScenarioExecution()
        self.tree = py_trees.composites.Sequence(name="", memory=True)

    def execute(self, scenario_content):
        parsed_tree = self.parser.parse_input_stream(InputStream(scenario_content))
        model = self.parser.create_internal_model(parsed_tree, self.tree, "test.osc", False)
        self.tree = create_py_tree(model, self.tree, self.parser.logger, False)
        self.scenario_execution_ros.tree = self.tree
        self.scenario_execution_ros.run()

    def tearDown(self):
        self.node.destroy_node()
        rclpy.try_shutdown()

    def service_callback(self, msg, response):
        self.request_received = msg.data
        return response

    def test_success(self):
        tree = self.parser.process_file(os.path.join(
            self.scenario_dir, 'scenarios', 'test', 'test_ros_service_call.osc'), False)
        self.scenario_execution_ros.tree = tree
        self.scenario_execution_ros.run()
        self.assertTrue(self.scenario_execution_ros.process_results())
        self.assertTrue(self.request_received)

    def test_success_repeat(self):
        scenario_content = """
import osc.helpers
import osc.ros

scenario test_ros_service_call:
    timeout(30s)
    do serial:
        repeat(2)
        service_call('/bla', 'std_srvs.srv.SetBool', '{\\\"data\\\": True}')
        emit end
"""
        self.execute(scenario_content)
        self.assertTrue(self.scenario_execution_ros.process_results())
