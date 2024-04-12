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
import threading

from ament_index_python.packages import get_package_share_directory

import rclpy
from std_msgs.msg import Bool

from scenario_execution.model.model_to_py_tree import create_py_tree
from scenario_execution_ros import ROSScenarioExecution
from scenario_execution.model.osc2_parser import OpenScenario2Parser
from scenario_execution.utils.logging import Logger

from antlr4.InputStream import InputStream


class TestCheckData(unittest.TestCase):
    # pylint: disable=missing-function-docstring,missing-class-docstring

    def setUp(self) -> None:
        rclpy.init()
        self.parser = OpenScenario2Parser(Logger('test', False))
        self.scenario_execution_ros = ROSScenarioExecution()

        self.scenario_dir = get_package_share_directory('scenario_execution_ros')

        self.received_msgs = []
        self.node = rclpy.create_node('test_node')
        self.srv = self.node.create_subscription(Bool, "/bla", self.callback, 10)

        self.executor = rclpy.executors.MultiThreadedExecutor()
        self.executor.add_node(self.node)
        self.executor_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.executor_thread.start()

    def tearDown(self):
        self.node.destroy_node()
        rclpy.try_shutdown()

    def callback(self, msg):
        self.received_msgs.append(msg)

    def test_success(self):
        scenario_content = """
import osc.ros

scenario test:
    do serial:
        parallel:
            test: serial:
                wait elapsed(1s)
                topic_publish() with:
                    keep(it.topic_name == '/bla')
                    keep(it.topic_type == 'std_msgs.msg.Bool')
                    keep(it.value == '{\\\"data\\\": True}')
            receive: serial:
                check_data(
                    topic_name: '/bla',
                    topic_type: 'std_msgs.msg.Bool',
                    variable_name: 'data',
                    expected_value: 'true'
                    fail_if_bad_comparison: true
                )
                emit end
        time_out: serial:
            wait elapsed(10s)
            emit fail
"""

        parsed_tree = self.parser.parse_input_stream(InputStream(scenario_content))
        model = self.parser.create_internal_model(parsed_tree, "test.osc", True)
        scenarios = create_py_tree(model, self.parser.logger, False)
        self.scenario_execution_ros.scenarios = scenarios
        self.scenario_execution_ros.run()
        self.assertTrue(self.scenario_execution_ros.process_results())
        
        