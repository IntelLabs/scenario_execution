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
import py_trees

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
        self.node = rclpy.create_node('test_node')
        self.tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self.node)
        self.timer = self.node.create_timer(0.1, self.publish_tf)
        self.timer = self.node.create_timer(0.1, self.publish_static_tf)
        self.timer = self.node.create_timer(0.1, self.publish_rotate_tf)
        self.time = 0.0
        self.executor = rclpy.executors.MultiThreadedExecutor()
        self.executor.add_node(self.node)
        self.executor_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.executor_thread.start()
        self.tree = py_trees.composites.Sequence()

    def execute(self, scenario_content):
        parsed_tree = self.parser.parse_input_stream(InputStream(scenario_content))
        model = self.parser.create_internal_model(parsed_tree, self.tree, "test.osc", False)
        create_py_tree(model, self.tree, self.parser.logger, False)
        self.scenario_execution_ros.tree = self.tree
        self.scenario_execution_ros.run()

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

    def publish_rotate_tf(self):
        self.time += 0.1
        static_transform_stamped = TransformStamped()
        static_transform_stamped.header.stamp = self.node.get_clock().now().to_msg()
        static_transform_stamped.header.frame_id = 'map'
        static_transform_stamped.child_frame_id = 'robot_rotating'
        # Update the child frame's position using a sinusoidal function
        amplitude = 1.0
        displacement = amplitude * math.sin(self.time)
        static_transform_stamped.transform.translation.x = 0.0
        static_transform_stamped.transform.translation.y = 0.0
        static_transform_stamped.transform.translation.z = 0.0
        static_transform_stamped.transform.rotation.x = displacement
        static_transform_stamped.transform.rotation.y = 0.0
        static_transform_stamped.transform.rotation.z = 0.0
        static_transform_stamped.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(static_transform_stamped)

    def tearDown(self):
        self.running = False
        self.node.destroy_node()
        rclpy.try_shutdown()

# REQUIRED PARAMETERS
    # frame_id: The frame ID to check for movement.
    # parent_frame_id: The parent frame ID against which the movement is checked.
    # timeout: The timeout duration without movement, in seconds.

# DEFAULT VALUES
    # threshold_translation: 0.01 mps (meters per second)
    # threshold_rotation: 0.01 radps (radians per second)
    # fail_on_finish: True
    # wait_for_first_transform: True
    # tf_topic_namespace: (optional)
    # use_sim_time: (optional)

# TESTS PERFORMED

# 1. Minimal Test:
    # Description: All default values remain; only frame names and timeout are specified.
    # Case 1: Test fails with timeout if there is no movement between frames.
    # Case 2: Test keeps running and ends with a scenario timeout as frames are moving with a threshold more than the default value.
    # Case 3: Test keeps running and ends with a scenario timeout if the provided frame does not exist.

# 2. Threshold Translation and Threshold Orientation:
    # Case 4: Test with threshold_translation set to 1 mps. The test fails with timeout as the average threshold of the robot_moving frame is less than 1 mps (meters per second).
    # Case 5: Test with threshold_rotation set to 5 radps. The test fails with timeout as the average threshold of the robot_rotating frame is less than 5 radps (radians per second).

# 3. fail_on_finish: False
    # Case 6: Test succeeds if no movement is observed between frames.

# 4. wait_for_first_transform: False
    # Case 7: Test fails if the provided frames do not exist.
    # Case 8: Test fails if frames exist but no movement is detected within the given timeout.
    # Case 9: Test keeps running if movement is detected within the given timeout and ends with a scenario timeout.

    def test_case_1(self):
        scenario_content = """
import osc.ros
scenario test_assert_tf_moving:
    do serial:
        assert_tf_moving(
            frame_id: 'robot',
            timeout: 10)
        emit end
"""
        self.execute(scenario_content)
        self.assertFalse(self.scenario_execution_ros.process_results())

    def test_case_2(self):
        scenario_content = """
import osc.ros
scenario test_assert_tf_moving:
    do parallel:
        serial:
            assert_tf_moving(
                frame_id: 'robot_moving',
                timeout: 10)
            emit fail
        time_out: serial:
            wait elapsed(12s)
            emit end
"""
        self.execute(scenario_content)
        self.assertTrue(self.scenario_execution_ros.process_results())

    def test_case_3(self):
        scenario_content = """
import osc.ros
scenario test_assert_tf_moving:
    do parallel:
        serial:
            assert_tf_moving(
                frame_id: 'robot_move',
                timeout: 10)
            emit end
        time_out: serial:
            wait elapsed(12s)
            emit fail
"""
        self.execute(scenario_content)
        self.assertFalse(self.scenario_execution_ros.process_results())

    def test_case_4(self):
        scenario_content = """
import osc.ros
scenario test_assert_tf_moving:
    do serial:
        assert_tf_moving(
            frame_id: 'robot_moving',
            threshold_translation: 1.0,
            timeout: 10)
        emit end
"""
        self.execute(scenario_content)
        self.assertFalse(self.scenario_execution_ros.process_results())

    def test_case_5(self):
        scenario_content = """
import osc.ros
scenario test_assert_tf_moving:
    do serial:
        assert_tf_moving(
            frame_id: 'robot_rotating',
            threshold_rotation: 5.0,
            timeout: 10)
        emit end
"""
        self.execute(scenario_content)
        self.assertFalse(self.scenario_execution_ros.process_results())

    def test_case_6(self):
        scenario_content = """
import osc.ros
scenario test_assert_tf_moving:
    do serial:
        assert_tf_moving(
            frame_id: 'robot',
            timeout: 10,
            fail_on_finish: false)
        emit end
"""
        self.execute(scenario_content)
        self.assertTrue(self.scenario_execution_ros.process_results())

    def test_case_7(self):
        scenario_content = """
import osc.ros
scenario test_assert_tf_moving:
    do serial:
        assert_tf_moving(
            frame_id: 'robot_leg',
            timeout: 10,
            wait_for_first_transform: false)
        emit end
"""
        self.execute(scenario_content)
        self.assertFalse(self.scenario_execution_ros.process_results())

    def test_case_8(self):
        scenario_content = """
import osc.ros
scenario test_assert_tf_moving:
    do serial:
        assert_tf_moving(
            frame_id: 'robot',
            timeout: 10,
            wait_for_first_transform: false)
        emit end
"""
        self.execute(scenario_content)
        self.assertFalse(self.scenario_execution_ros.process_results())

    def test_case_9(self):
        scenario_content = """
import osc.ros
scenario test_assert_tf_moving:
    do parallel:
        serial:
            assert_tf_moving(
                frame_id: 'robot_moving',
                timeout: 10,
                wait_for_first_transform: false)
            emit fail
        time_out: serial:
            wait elapsed(12s)
            emit end
"""
        self.execute(scenario_content)
        self.assertTrue(self.scenario_execution_ros.process_results())
