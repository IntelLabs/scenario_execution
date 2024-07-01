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
import py_trees
import rclpy

from scenario_execution_ros import ROSScenarioExecution
from scenario_execution.model.osc2_parser import OpenScenario2Parser
from scenario_execution.model.model_to_py_tree import create_py_tree
from scenario_execution.utils.logging import Logger

from antlr4.InputStream import InputStream


class TestCheckData(unittest.TestCase):
    # pylint: disable=missing-function-docstring,missing-class-docstring

    def setUp(self) -> None:
        rclpy.init()
        self.parser = OpenScenario2Parser(Logger('test', False))
        self.scenario_execution_ros = ROSScenarioExecution()
        self.tree = py_trees.composites.Sequence()

    def execute(self, scenario_content):
        parsed_tree = self.parser.parse_input_stream(InputStream(scenario_content))
        model = self.parser.create_internal_model(parsed_tree, self.tree, "test.osc", False)
        create_py_tree(model, self.tree, self.parser.logger, False)
        self.scenario_execution_ros.tree = self.tree
        self.scenario_execution_ros.live_tree = True
        self.scenario_execution_ros.run()

    def tearDown(self):
        rclpy.try_shutdown()

    def test_fail_no_msgs(self):
        scenario_content = """
import osc.ros

scenario test:
    do parallel:
        receive: serial:
            check_data(
                topic_name: '/bla',
                topic_type: 'std_msgs.msg.Bool',
                member_name: 'data',
                expected_value: 'True')
            emit end
        time_out: serial:
            wait elapsed(3s)
            emit fail
"""
        self.execute(scenario_content)
        self.assertFalse(self.scenario_execution_ros.process_results())

    def test_sucess_field(self):
        scenario_content = """
import osc.ros

scenario test:
    do parallel:
        test: serial:
            wait elapsed(1s)
            topic_publish(
                topic_name: '/bla',
                topic_type: 'std_msgs.msg.Bool',
                value: '{\\\"data\\\": True}')
        receive: serial:
            check_data(
                topic_name: '/bla',
                topic_type: 'std_msgs.msg.Bool',
                member_name: 'data',
                expected_value: 'True')
            emit end
        time_out: serial:
            wait elapsed(10s)
            emit fail
"""
        self.execute(scenario_content)
        self.assertTrue(self.scenario_execution_ros.process_results())

    def test_error_empty_member_name(self):
        scenario_content = """
import osc.ros

scenario test:
    do parallel:
        test: serial:
            wait elapsed(1s)
            topic_publish(
                topic_name: '/bla',
                topic_type: 'std_msgs.msg.Bool',
                value: '{\\\"data\\\": True}')
        receive: serial:
            check_data(
                topic_name: '/bla',
                topic_type: 'std_msgs.msg.Bool',
                expected_value: '{\\\"data\\\": True}')
            emit end
        time_out: serial:
            wait elapsed(10s)
            emit fail
"""
        self.execute(scenario_content)
        self.assertTrue(self.scenario_execution_ros.process_results())

    def test_error_empty_member_name_comparison_fails(self):
        scenario_content = """
import osc.ros

scenario test:
    do parallel:
        test: serial:
            wait elapsed(1s)
            topic_publish(
                topic_name: '/bla',
                topic_type: 'std_msgs.msg.Bool',
                value: '{\\\"data\\\": True}')
        receive: serial:
            check_data(
                topic_name: '/bla',
                topic_type: 'std_msgs.msg.Bool',
                expected_value: '{\\\"data\\\": False}')
            emit end
        time_out: serial:
            wait elapsed(10s)
            emit fail
"""
        self.execute(scenario_content)
        self.assertFalse(self.scenario_execution_ros.process_results())

    def test_fail_if_bad_comparison(self):
        scenario_content = """
import osc.ros

scenario test:
    do parallel:
        test: serial:
            wait elapsed(1s)
            topic_publish(
                topic_name: '/bla',
                topic_type: 'std_msgs.msg.Bool',
                value: '{\\\"data\\\": True}')
        receive: serial:
            check_data(
                topic_name: '/bla',
                topic_type: 'std_msgs.msg.Bool',
                member_name: 'data',
                expected_value: 'false',
                fail_if_bad_comparison: true)
            emit end
        time_out: serial:
            wait elapsed(10s)
            emit end
"""
        self.execute(scenario_content)
        self.assertFalse(self.scenario_execution_ros.process_results())

    def test_wait_for_comparison_to_succeed(self):
        scenario_content = """
import osc.ros

scenario test:
    do parallel:
        test: serial:
            wait elapsed(1s)
            topic_publish(
                topic_name: '/bla',
                topic_type: 'std_msgs.msg.Bool',
                value: '{\\\"data\\\": True}')
            wait elapsed(3s)
            topic_publish(
                topic_name: '/bla',
                topic_type: 'std_msgs.msg.Bool',
                value: '{\\\"data\\\": False}')
        receive: serial:
            check_data(
                topic_name: '/bla',
                topic_type: 'std_msgs.msg.Bool',
                member_name: 'data',
                expected_value: false)
            emit end
        time_out: serial:
            wait elapsed(10s)
            emit fail
"""
        self.execute(scenario_content)
        self.assertTrue(self.scenario_execution_ros.process_results())

    def test_wait_for_first_message_false_data_available(self):
        scenario_content = """
import osc.ros

scenario test:
    do parallel:
        serial:
            topic_publish(
                topic_name: '/bla',
                topic_type: 'std_msgs.msg.Bool',
                value: '{\\\"data\\\": True}')
            wait elapsed(1s)
            check_data(
                topic_name: '/bla',
                topic_type: 'std_msgs.msg.Bool',
                member_name: 'data',
                expected_value: 'True',
                wait_for_first_message: false)
            emit end
        time_out: serial:
            wait elapsed(10s)
            emit fail
"""
        self.execute(scenario_content)
        self.assertTrue(self.scenario_execution_ros.process_results())

    def test_wait_for_first_message_false_data_not_available(self):
        scenario_content = """
import osc.ros

scenario test:
    do parallel:
        serial:
            check_data(
                topic_name: '/bla',
                topic_type: 'std_msgs.msg.Bool',
                member_name: 'data',
                expected_value: 'True',
                wait_for_first_message: false)
            emit end
        time_out: serial:
            wait elapsed(3s)
            emit fail
"""
        self.execute(scenario_content)
        self.assertFalse(self.scenario_execution_ros.process_results())

    def test_wait_for_first_message_true_data_gets_available(self):
        scenario_content = """
import osc.ros

scenario test:
    do parallel:
        serial:
            wait elapsed(1s)
            topic_publish(
                topic_name: '/bla',
                topic_type: 'std_msgs.msg.Bool',
                value: '{\\\"data\\\": True}')
        serial:
            check_data(
                topic_name: '/bla',
                topic_type: 'std_msgs.msg.Bool',
                member_name: 'data',
                expected_value: 'True',
                wait_for_first_message: true)
            emit end
        time_out: serial:
            wait elapsed(3s)
            emit fail
"""
        self.execute(scenario_content)
        self.assertTrue(self.scenario_execution_ros.process_results())
