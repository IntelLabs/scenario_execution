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
import numpy as np
import tf2_ros
from scenario_execution_ros.actions.nav2_common import NamespacedTransformListener
from tf2_ros import TransformException  # pylint: disable= no-name-in-module


class AssertTfMoving(py_trees.behaviour.Behaviour):

    def __init__(self, name, frame_id: str, parent_frame_id: str, timeout: int, threshold_speed: bool, fail_on_finish: bool, wait_for_first_transform: bool, namespace: str, sim: bool):
        super().__init__(name)
        self.frame_id = frame_id
        self.parent_frame_id = parent_frame_id
        self.timeout = timeout
        self.fail_on_finish = fail_on_finish
        self.threshold_speed = threshold_speed
        self.wait_for_first_transform = wait_for_first_transform
        self.namespace = namespace
        self.sim = sim
        self.node = None
        self.displacement = True
        self.transforms_received = 0
        self.max_transforms = 5
        self.prev_transforms = []
        self.start_timer = 0

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

        self.get_transform(self.frame_id, self.parent_frame_id)

    def update(self) -> py_trees.common.Status:
        result = py_trees.common.Status.FAILURE
        transform, Success = self.get_transform(self.parent_frame_id, self.frame_id)
        if self.wait_for_first_transform:
            if not Success:
                self.logger.info(f"Waiting from the first tranformation on frame {self.frame_id}")
                self.feedback_message = f"Waiting for first tranformation on frame {self.frame_id}"
                result = py_trees.common.Status.RUNNING
            else:
                self.wait_for_first_transform = False
                result = py_trees.common.Status.RUNNING
        else:
            average_displacement = self.calculated_displacement(transform)
            if average_displacement is None:
                return py_trees.common.Status.RUNNING
            elif average_displacement == 0:
                if self.displacement:
                    self.start_time = time.time()
                    self.displacement = False
                elif time.time() - self.start_time > self.timeout:
                    self.logger.error("Timeout: No movement detected for {} seconds.".format(self.timeout))
                    self.feedback_message = f"Timeout: No movement detected for {self.timeout} seconds."
                    return py_trees.common.Status.FAILURE
                self.feedback_message = "Frame is not moving."
                return py_trees.common.Status.RUNNING
            elif self.fail_on_finish and (average_displacement > self.threshold_speed):
                self.feedback_message = f"The movement threshold of frame {self.frame_id} with respect to frame {self.parent_frame_id} ({average_displacement}) exceeded."
                return py_trees.common.Status.FAILURE
            elif average_displacement > self.threshold_speed:
                self.feedback_message = f"The movement threshold of frame {self.frame_id} with respect to frame {self.parent_frame_id} ({average_displacement}) exceeded."
                self.logger.info(
                    f"The movement threshold of frame {self.frame_id} with respect to frame {self.parent_frame_id} ({average_displacement}) exceeded.")
                return py_trees.common.Status.SUCCESS
            else:
                self.feedback_message = f"Avergae Threshold: {average_displacement}"
                return py_trees.common.Status.RUNNING
        return result

    def get_transform(self, frame_id, parent_frame_id):
        when = self.node.get_clock().now()
        if self.sim:
            when = rclpy.time.Time()
        try:
            transform = self.tf_buffer.lookup_transform(
                parent_frame_id,
                frame_id,
                when,
                timeout=rclpy.duration.Duration(seconds=1.0),
            )
            self.feedback_message = f"Transform {parent_frame_id} -> {frame_id} got available."  # pylint: disable= attribute-defined-outside-init
            return transform, True
        except TransformException as ex:
            self.feedback_message = f"Could not {frame_id} and {parent_frame_id}"  # pylint: disable= attribute-defined-outside-init
            self.node.get_logger().warn(
                f'Could not transform {frame_id} and {parent_frame_id} at time {when}: {ex}')
            return None, False

    def calculated_displacement(self, transform):
        if self.transforms_received < self.max_transforms:
            self.prev_transforms.append(transform)
            self.transforms_received += 1
            return

        for prev_transform in self.prev_transforms:
            prev_translation = np.array([prev_transform.transform.translation.x,
                                         prev_transform.transform.translation.y,
                                         prev_transform.transform.translation.z])
            current_translation = np.array([transform.transform.translation.x,
                                            transform.transform.translation.y,
                                            transform.transform.translation.z])
            average_displacement = np.linalg.norm(prev_translation - current_translation)
        self.prev_transforms.pop(0)
        self.prev_transforms.append(transform)
        return average_displacement
