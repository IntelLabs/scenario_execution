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
import threading

from ament_index_python.packages import get_package_share_directory

import rclpy
import rclpy.executors
import rclpy.time
from scenario_execution_ros import ROSScenarioExecution
from scenario_execution.model.osc2_parser import OpenScenario2Parser
from scenario_execution.model.model_to_py_tree import create_py_tree
from scenario_execution.utils.logging import Logger
from antlr4.InputStream import InputStream
from geometry_msgs.msg import TransformStamped
import math
import tf2_ros


class TestScenarioExecutionSuccess(unittest.TestCase):

    def setUp(self) -> None:
        rclpy.init()
        self.running = True
        self.parser = OpenScenario2Parser(Logger('test', False))
        self.scenario_execution_ros = ROSScenarioExecution()
        self.scenario_dir = get_package_share_directory('scenario_execution_ros')
        self.node = rclpy.create_node('test_node')
        self.tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self.node)
        self.timer = self.node.create_timer(0.1, self.publish_tf)
        self.timer = self.node.create_timer(0.1, self.publish_static_tf)
        self.time = 0.0
        self.executor = rclpy.executors.MultiThreadedExecutor()
        self.executor.add_node(self.node)
        self.executor_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.executor_thread.start()

    def publish_static_tf(self):
        static_transform_stamped = TransformStamped()
        static_transform_stamped.header.stamp = self.node.get_clock().now().to_msg()
        static_transform_stamped.header.frame_id = 'map'
        static_transform_stamped.child_frame_id = 'robot'
        static_transform_stamped.transform.translation.x = 1.0
        static_transform_stamped.transform.translation.y = 0.0
        static_transform_stamped.transform.translation.z = 0.0
        static_transform_stamped.transform.rotation.x = 0.0
        static_transform_stamped.transform.rotation.y = 0.0
        static_transform_stamped.transform.rotation.z = 0.0
        static_transform_stamped.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(static_transform_stamped)

    def publish_tf(self):
        self.time += 0.1
        static_transform_stamped = TransformStamped()
        static_transform_stamped.header.stamp = self.node.get_clock().now().to_msg()
        static_transform_stamped.header.frame_id = 'map'
        static_transform_stamped.child_frame_id = 'robot_moving'
        # Update the child frame's position using a sinusoidal function
        amplitude = 1.0
        displacement = amplitude * math.sin(self.time)
        static_transform_stamped.transform.translation.x = displacement
        static_transform_stamped.transform.translation.y = 0.0
        static_transform_stamped.transform.translation.z = 0.0
        static_transform_stamped.transform.rotation.x = 0.0
        static_transform_stamped.transform.rotation.y = 0.0
        static_transform_stamped.transform.rotation.z = 0.0
        static_transform_stamped.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(static_transform_stamped)

    def tearDown(self):
        self.running = False
        self.node.destroy_node()
        rclpy.try_shutdown()

    def test_success(self):
        scenario_content = """
import osc.ros
scenario test_assert_tf_moving:
    do parallel:
        serial:
            assert_tf_moving(
                frame_id: 'robot',
                timeout: 10,
                fail_on_finish: false)
            emit end
"""
        parsed_tree = self.parser.parse_input_stream(InputStream(scenario_content))
        model = self.parser.create_internal_model(parsed_tree, "test.osc", False)
        scenarios = create_py_tree(model, self.parser.logger, False)
        self.scenario_execution_ros.scenarios = scenarios
        self.scenario_execution_ros.run()
        self.assertTrue(self.scenario_execution_ros.process_results())

    def test_fail_on_finish(self):
        scenario_content = """
import osc.ros
scenario test_assert_tf_moving:
    do parallel:
        serial:
            assert_tf_moving(
                frame_id: 'robot',
                timeout: 10)
            emit end
"""
        parsed_tree = self.parser.parse_input_stream(InputStream(scenario_content))
        model = self.parser.create_internal_model(parsed_tree, "test.osc", False)
        scenarios = create_py_tree(model, self.parser.logger, False)
        self.scenario_execution_ros.scenarios = scenarios
        self.scenario_execution_ros.run()
        self.assertFalse(self.scenario_execution_ros.process_results())

    def test_running(self):
        scenario_content = """
import osc.ros
scenario test_assert_tf_moving:
    do parallel:
        serial:
            assert_tf_moving(
                frame_id: 'robot_moving',
                timeout: 5)
            emit end
        time_out: serial:
            wait elapsed(10s)
            emit fail
"""
        parsed_tree = self.parser.parse_input_stream(InputStream(scenario_content))
        model = self.parser.create_internal_model(parsed_tree, "test.osc", False)
        scenarios = create_py_tree(model, self.parser.logger, False)
        self.scenario_execution_ros.scenarios = scenarios
        self.scenario_execution_ros.run()
        self.assertFalse(self.scenario_execution_ros.process_results())
