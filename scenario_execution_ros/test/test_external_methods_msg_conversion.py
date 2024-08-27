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
import threading
from scenario_execution_ros import ROSScenarioExecution
from scenario_execution.model.osc2_parser import OpenScenario2Parser
from scenario_execution.model.model_to_py_tree import create_py_tree
from scenario_execution.model.model_blackboard import create_py_tree_blackboard
from scenario_execution.utils.logging import Logger
from antlr4.InputStream import InputStream

os.environ["PYTHONUNBUFFERED"] = '1'


class TestExternalMethodsMsgConversion(unittest.TestCase):
    # pylint: disable=missing-function-docstring

    def setUp(self):
        rclpy.init()
        self.parser = OpenScenario2Parser(Logger('test', False))
        self.scenario_execution_ros = ROSScenarioExecution()
        self.tree = py_trees.composites.Sequence(name="", memory=True)

    def execute(self, scenario_content):
        parsed_tree = self.parser.parse_input_stream(InputStream(scenario_content))
        model = self.parser.create_internal_model(parsed_tree, self.tree, "test.osc", False)
        create_py_tree_blackboard(model, self.tree, self.parser.logger, False)
        self.tree = create_py_tree(model, self.tree, self.parser.logger, False)
        self.scenario_execution_ros.tree = self.tree
        self.scenario_execution_ros.run()

    def tearDown(self):
        rclpy.try_shutdown()

    def test_success(self):
        scenario_content = """
import osc.ros
import osc.helpers

struct current_state:
    var test_pose: string

scenario test_success:
    timeout(10s)
    current: current_state
    
    do parallel:
        topic_monitor('/test_pose', 'geometry_msgs.msg.PoseWithCovarianceStamped', current.test_pose)
        serial:
            wait elapsed(0.5s)
            topic_publish('/test_pose', 'geometry_msgs.msg.PoseWithCovarianceStamped', "{ \\\'pose\\\': { \\\'pose\\\': { \\\'position\\\': { \\\'x\\\': 42 }}}}")
        serial:
            wait elapsed(3s)
            topic_publish('/pose_only', 'geometry_msgs.msg.Pose', msg_conversion.get_object_member(current.test_pose, "pose.pose"))

        serial:
            check_data('/pose_only', 'geometry_msgs.msg.Pose', expected_value: '{ \\\"position\\\": { \\\"x\\\": 42 }}')
            emit end
"""
        self.execute(scenario_content)
        self.assertTrue(self.scenario_execution_ros.process_results())
