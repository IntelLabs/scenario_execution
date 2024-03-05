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

""" Test for spawn and exists """
import os
import unittest
import threading

from ament_index_python.packages import get_package_share_directory

import rclpy
from rcl_interfaces.msg import SetParametersResult

from scenario_execution import ROSScenarioExecution
from scenario_execution_base.model.osc2_parser import OpenScenario2Parser
from scenario_execution_base.utils.logging import Logger


class TestRosSetNodeParameter(unittest.TestCase):
    # pylint: disable=missing-function-docstring,missing-class-docstring
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.parser = OpenScenario2Parser(Logger('test'))
        self.scenario_execution = ROSScenarioExecution()
        self.scenario_dir = get_package_share_directory('scenario_execution')

        self.node = rclpy.create_node('test_node')
        self.node.declare_parameter('testBoolParam', False)
        self.node.declare_parameter('testFloatParam', 0.)
        self.bool_value = self.node.get_parameter('testBoolParam').value
        self.float_value = self.node.get_parameter('testFloatParam').value
        self.node.add_on_set_parameters_callback(self.callback)
        # self.srv = self.node.create_service(SetBool, "/bla", self.service_callback)

        self.executor = rclpy.executors.MultiThreadedExecutor()
        self.executor.add_node(self.node)
        self.executor_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.executor_thread.start()

    def tearDown(self):
        self.node.destroy_node()

    def callback(self, params):
        print(f"set_parameters callback called.")
        for param in params:
            print(f"  {param.name}: {param.value}")
            if param.name == "testBoolParam":
                self.bool_value = param.value
            if param.name == "testFloatParam":
                self.float_value = param.value
        return SetParametersResult(successful=True)

    def test_success(self):
        scenarios = self.parser.process_file(os.path.join(
            self.scenario_dir, 'scenarios', 'test', 'test_ros_set_node_parameter.osc'), False)
        self.assertIsNotNone(scenarios)
        self.scenario_execution.scenarios = scenarios
        ret = self.scenario_execution.run()
        self.assertTrue(ret)
        self.assertTrue(self.bool_value)
        self.assertEqual(self.float_value, 3.14)
