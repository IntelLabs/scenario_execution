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
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import GoalEvent
from rclpy.callback_groups import ReentrantCallbackGroup

import threading
from scenario_execution_ros import ROSScenarioExecution
from scenario_execution.model.osc2_parser import OpenScenario2Parser
from scenario_execution.model.model_to_py_tree import create_py_tree
from scenario_execution.utils.logging import Logger
from ament_index_python.packages import get_package_share_directory
from antlr4.InputStream import InputStream

from example_interfaces.action import Fibonacci


import time

os.environ["PYTHONUNBUFFERED"] = '1'


class TestScenarioExectionSuccess(unittest.TestCase):
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

        self.parser = OpenScenario2Parser(Logger('test', False))
        self.scenario_execution_ros = ROSScenarioExecution()

        self.goal_callback_reponse = GoalResponse.ACCEPT
        self.cancel_callback_reponse = CancelResponse.ACCEPT
        self.goal_response = GoalEvent.SUCCEED
        self.action_server = ActionServer(
            self.node,
            Fibonacci,
            '/test_action',
            execute_callback=self.execute_callback,
            callback_group=ReentrantCallbackGroup(),
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback)

    def goal_callback(self, goal_request):
        return self.goal_callback_reponse

    def cancel_callback(self, goal_handle):
        return self.cancel_callback_reponse

    def execute_callback(self, goal_handle):
        feedback_msg = Fibonacci.Feedback()

        for _ in range(1, 3):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return Fibonacci.Result()
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(1)

        if self.goal_response == GoalEvent.SUCCEED:
            goal_handle.succeed()
        elif self.goal_response == GoalEvent.ABORT:
            goal_handle.abort()
        else:
            goal_handle.cancel()

        result = Fibonacci.Result()
        result.sequence = feedback_msg.sequence

        return result

    def tearDown(self):
        self.node.destroy_node()
        rclpy.try_shutdown()

    def test_success(self):
        scenario_content = """
import osc.ros

scenario test:
    do parallel:
        serial:
            wait elapsed(1s)
            action_call(action_name: "/test_action", action_type: "example_interfaces.action.Fibonacci", data: '{\\"order\\": 3}')
            emit end
        time_out: serial:
            wait elapsed(10s)
            emit fail
"""
        parsed_tree = self.parser.parse_input_stream(InputStream(scenario_content))
        model = self.parser.create_internal_model(parsed_tree, "test.osc", False)
        scenarios = create_py_tree(model, self.parser.logger, True)
        self.scenario_execution_ros.scenarios = scenarios
        self.scenario_execution_ros.run()
        self.assertTrue(self.scenario_execution_ros.process_results())

    def test_goal_not_accepted(self):
        scenario_content = """
import osc.ros

scenario test:
    do parallel:
        serial:
            wait elapsed(1s)
            action_call(action_name: "/test_action", action_type: "example_interfaces.action.Fibonacci", data: '{\\"order\\": 3}')
            emit end
        time_out: serial:
            wait elapsed(10s)
            emit fail
"""
        self.goal_callback_reponse = GoalResponse.REJECT
        parsed_tree = self.parser.parse_input_stream(InputStream(scenario_content))
        model = self.parser.create_internal_model(parsed_tree, "test.osc", False)
        scenarios = create_py_tree(model, self.parser.logger, True)
        self.scenario_execution_ros.scenarios = scenarios
        self.scenario_execution_ros.run()
        self.assertFalse(self.scenario_execution_ros.process_results())

    def test_action_aborted(self):
        scenario_content = """
import osc.ros

scenario test:
    do parallel:
        serial:
            wait elapsed(1s)
            action_call(action_name: "/test_action", action_type: "example_interfaces.action.Fibonacci", data: '{\\"order\\": 3}')
            emit end
        time_out: serial:
            wait elapsed(10s)
            emit fail
"""
        self.goal_response = GoalEvent.ABORT
        parsed_tree = self.parser.parse_input_stream(InputStream(scenario_content))
        model = self.parser.create_internal_model(parsed_tree, "test.osc", False)
        scenarios = create_py_tree(model, self.parser.logger, True)
        self.scenario_execution_ros.scenarios = scenarios
        self.scenario_execution_ros.run()
        self.assertFalse(self.scenario_execution_ros.process_results())

    def test_action_canceled(self):
        scenario_content = """
import osc.ros

scenario test:
    do parallel:
        serial:
            wait elapsed(1s)
            action_call(action_name: "/test_action", action_type: "example_interfaces.action.Fibonacci", data: '{\\"order\\": 3}')
            emit end
        time_out: serial:
            wait elapsed(10s)
            emit fail
"""
        self.goal_response = GoalEvent.CANCELED
        parsed_tree = self.parser.parse_input_stream(InputStream(scenario_content))
        model = self.parser.create_internal_model(parsed_tree, "test.osc", False)
        scenarios = create_py_tree(model, self.parser.logger, True)
        self.scenario_execution_ros.scenarios = scenarios
        self.scenario_execution_ros.run()
        self.assertFalse(self.scenario_execution_ros.process_results())

    def test_invalid_type(self):
        scenario_content = """
import osc.ros

scenario test:
    do parallel:
        serial:
            wait elapsed(1s)
            action_call(action_name: "/test_action", action_type: "UNKNOWN", data: '{\\"order\\": 3}')
            emit end
        time_out: serial:
            wait elapsed(10s)
            emit fail
"""
        parsed_tree = self.parser.parse_input_stream(InputStream(scenario_content))
        model = self.parser.create_internal_model(parsed_tree, "test.osc", False)
        scenarios = create_py_tree(model, self.parser.logger, True)
        self.scenario_execution_ros.scenarios = scenarios
        self.scenario_execution_ros.run()
        self.assertFalse(self.scenario_execution_ros.process_results())