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

import rclpy
import py_trees  # pylint: disable=import-error
from rclpy.node import Node
import time
import tf2_ros
from scenario_execution_ros.actions.nav2_common import NamespacedTransformListener
from tf2_ros import TransformException  # pylint: disable= no-name-in-module
import math


class AssertTfMoving(py_trees.behaviour.Behaviour):

    def __init__(self, name, frame_id: str, parent_frame_id: str, timeout: int, threshold_translation: float, threshold_orientation: float, fail_on_finish: bool, wait_for_first_transform: bool, namespace: str, use_sim_time: bool):
        super().__init__(name)
        self.frame_id = frame_id
        self.parent_frame_id = parent_frame_id
        self.timeout = timeout
        self.fail_on_finish = fail_on_finish
        self.threshold_translation = threshold_translation
        self.threshold_orientation = threshold_orientation
        self.wait_for_first_transform = wait_for_first_transform
        self.namespace = namespace
        self.use_sim_time = use_sim_time
        self.start_timeout = False
        self.timer = 0
        self.transforms_received = 0
        self.prev_transform = None
        self.node = None
        self.tf_buffer = None
        self.tf_listener = None

    def setup(self, **kwargs):
        try:
            self.node: Node = kwargs['node']
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(
                self.name, self.__class__.__name__)
            raise KeyError(error_message) from e

        self.feedback_message = f"Waiting for transform {self.parent_frame_id} --> {self.frame_id}"  # pylint: disable= attribute-defined-outside-init
        self.tf_buffer = tf2_ros.Buffer()
        tf_prefix = self.namespace
        if not tf_prefix.startswith('/') and tf_prefix != '':
            tf_prefix = "/" + tf_prefix
        self.tf_listener = NamespacedTransformListener(
            node=self.node,
            buffer=self.tf_buffer,
            tf_topic=(tf_prefix + "/tf"),
            tf_static_topic=(tf_prefix + "/tf_static"),
        )

    def update(self) -> py_trees.common.Status:
        now = time.time()
        transform = self.get_transform(self.frame_id, self.parent_frame_id)
        result = py_trees.common.Status.RUNNING
        if self.wait_for_first_transform:
            if transform is not None:
                self.feedback_message = f"Transform {self.parent_frame_id} -> {self.frame_id} got available."  # pylint: disable= attribute-defined-outside-init
                self.prev_transform = transform
                self.timer = time.time()
                self.wait_for_first_transform = False
                result = py_trees.common.Status.RUNNING
            else:
                self.feedback_message = f"Waiting for first tranformation {self.parent_frame_id} -> {self.frame_id}"  # pylint: disable= attribute-defined-outside-init
                result = py_trees.common.Status.RUNNING
        else:
            delta_time = now - self.timer
            translational_speed, rotational_speed = self.calculated_displacement(transform, self.prev_transform, delta_time)
            self.prev_transform = transform
            if translational_speed >= self.threshold_translation or rotational_speed >= self.threshold_orientation:
                self.start_timeout = False
                self.timer = time.time()
                self.feedback_message = f"The frame {self.frame_id} is moving with respect to frame {self.parent_frame_id} with linear velocity ({translational_speed}) and rotational ({rotational_speed})."  # pylint: disable= attribute-defined-outside-init
                result = py_trees.common.Status.RUNNING
            else:
                if not self.start_timeout:
                    self.timer = time.time()
                    self.start_timeout = True
                elif now - self.timer > self.timeout and self.fail_on_finish:
                    self.feedback_message = f"Timeout: No movement detected for {self.timeout} seconds."  # pylint: disable= attribute-defined-outside-init
                    result = py_trees.common.Status.FAILURE
                elif now - self.timer > self.timeout and not self.fail_on_finish:
                    self.logger.error("Timeout: No movement detected for {} seconds.".format(self.timeout))
                    self.feedback_message = f"Timeout: No movement detected for {self.timeout} seconds."  # pylint: disable= attribute-defined-outside-init
                    result = py_trees.common.Status.SUCCESS
                else:
                    self.feedback_message = "Frame is not moving."  # pylint: disable= attribute-defined-outside-init
                    result = py_trees.common.Status.RUNNING
        return result

    def get_transform(self, frame_id, parent_frame_id):
        when = self.node.get_clock().now()
        if self.use_sim_time:
            when = rclpy.time.Time()
        try:
            transform = self.tf_buffer.lookup_transform(
                parent_frame_id,
                frame_id,
                when,
                timeout=rclpy.duration.Duration(seconds=1.0),
            )
            return transform
        except TransformException as ex:
            self.node.get_logger().warn(f'Could not transform {frame_id} and {parent_frame_id} at time {when}: {ex}')
            return None

    def calculated_displacement(self, transform, prev_transform, delta_time):
        dx = prev_transform.transform.translation.x - transform.transform.translation.x
        dy = prev_transform.transform.translation.y - transform.transform.translation.y
        dz = prev_transform.transform.translation.z - transform.transform.translation.z
        translational_speed = math.sqrt(dx**2 + dy**2 + dz**2) / delta_time

        qx1, qy1, qz1, qw1 = prev_transform.transform.rotation.x, prev_transform.transform.rotation.y, prev_transform.transform.rotation.z, prev_transform.transform.rotation.w
        qx2, qy2, qz2, qw2 = transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w

        dqx, dqy, dqz, dqw = qx2 - qx1, qy2 - qy1, qz2 - qz1, qw2 - qw1
        rotational_speed = math.sqrt(dqx**2 + dqy**2 + dqz**2 + dqw**2) / delta_time

        return translational_speed, rotational_speed
