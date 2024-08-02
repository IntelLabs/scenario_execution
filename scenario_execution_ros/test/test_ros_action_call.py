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
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import GoalEvent
from rclpy.callback_groups import ReentrantCallbackGroup

import threading
from scenario_execution_ros import ROSScenarioExecution
from scenario_execution.model.osc2_parser import OpenScenario2Parser
from scenario_execution.model.model_to_py_tree import create_py_tree
from scenario_execution.utils.logging import Logger
from antlr4.InputStream import InputStream

from example_interfaces.action import Fibonacci


import time

os.environ["PYTHONUNBUFFERED"] = '1'


class TestRosActionCall(unittest.TestCase):
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
        self.tree = py_trees.composites.Sequence(name="", memory=True)

    def execute(self, scenario_content):
        parsed_tree = self.parser.parse_input_stream(InputStream(scenario_content))
        model = self.parser.create_internal_model(parsed_tree, self.tree, "test.osc", False)
        self.tree = create_py_tree(model, self.tree, self.parser.logger, False)
        self.scenario_execution_ros.tree = self.tree
        self.scenario_execution_ros.run()

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
        self.executor_thread.join()

    def test_success(self):
        scenario_content = """
import osc.helpers
import osc.ros

scenario test:
    timeout(10s)
    do serial:
        wait elapsed(1s)
        action_call(action_name: "/test_action", action_type: "example_interfaces.action.Fibonacci", data: '{\\"order\\": 3}')
"""
        self.execute(scenario_content)
        self.assertTrue(self.scenario_execution_ros.process_results())

    def test_goal_not_accepted(self):
        scenario_content = """
import osc.helpers
import osc.ros

scenario test:
    timeout(10s)
    do serial:
        wait elapsed(1s)
        action_call(action_name: "/test_action", action_type: "example_interfaces.action.Fibonacci", data: '{\\"order\\": 3}')
"""
        self.goal_callback_reponse = GoalResponse.REJECT
        self.execute(scenario_content)
        self.assertFalse(self.scenario_execution_ros.process_results())

    def test_action_aborted(self):
        scenario_content = """
import osc.helpers
import osc.ros

scenario test:
    timeout(10s)
    do serial:
        wait elapsed(1s)
        action_call(action_name: "/test_action", action_type: "example_interfaces.action.Fibonacci", data: '{\\"order\\": 3}')
"""
        self.goal_response = GoalEvent.ABORT
        self.execute(scenario_content)
        self.assertFalse(self.scenario_execution_ros.process_results())

    def test_action_canceled(self):
        scenario_content = """
import osc.helpers
import osc.ros

scenario test:
    timeout(10s)
    do serial:
        wait elapsed(1s)
        action_call(action_name: "/test_action", action_type: "example_interfaces.action.Fibonacci", data: '{\\"order\\": 3}')
"""
        self.goal_response = GoalEvent.CANCELED
        self.execute(scenario_content)
        self.assertFalse(self.scenario_execution_ros.process_results())

    def test_invalid_type(self):
        scenario_content = """
import osc.helpers
import osc.ros

scenario test:
    timeout(10s)
    do serial:
        wait elapsed(1s)
        action_call(action_name: "/test_action", action_type: "UNKNOWN", data: '{\\"order\\": 3}')
"""
        self.execute(scenario_content)
        self.assertFalse(self.scenario_execution_ros.process_results())
