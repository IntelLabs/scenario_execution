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


class AssertTfMoving(py_trees.behaviour.Behaviour):

    def __init__(self, name, frame_id: str, parent_frame_id: str, timeout: int, threshold_speed: bool, fail_on_finish: bool, wait_for_first_transform: bool):
        super().__init__(name)
        self.frame_id = frame_id
        self.parent_frame_id = parent_frame_id
        self.timeout = timeout
        self.fail_on_finish = fail_on_finish
        self.threshold_speed = threshold_speed
        self.wait_for_first_transform = wait_for_first_transform
        self.node = None
        self.average_displacement = None
        # self.tf_buffer = None
        # self.tf_listner = None

    def setup(self, **kwargs):
        try:
            self.node: Node = kwargs['node']
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(
                self.name, self.__class__.__name__)
            raise KeyError(error_message) from e

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listner = tf2_ros.TransformListener(buffer=self.tf_buffer, node=self.node)
        
        self.get_transform(self.frame_id, self.parent_frame_id)
    def update(self) -> py_trees.common.Status:

        result = py_trees.common.Status.FAILURE
        if self.wait_for_first_transform:
            self.logger.info(f"Waiting from the first tranformation on frame {self.frame_id}")
            self.feedback_message = f"Waiting for first tranformation on frame {self.frame_id}"
            result = py_trees.common.Status.RUNNING
        else:
            if self.fail_on_finish and (self.average_displacement > self.threshold_speed):
                result = py_trees.common.Status.FAILURE
                self.feedback_message = f"Threshold of frame {self.frame_id} with respect to frame {self.parent} is {self.average_displacement}"
            elif self.average_displacement > self.threshold_speed:
                result = py_trees.common.Status.SUCCESS
                self.feedback_message = f"Threshold of frame {self.frame_id} with respect to frame {self.parent_frame_id} is {self.average_displacement}"
            else:
                result = py_trees.common.Status.RUNNING
                self.feedback_message = f"Avergae Threshold: {self.average_displacement}"
        return result

    def get_transform(self, frame_id, parent_frame_id):
        try:
            when = self.node.get_clock().now()
            transform = self.tf_buffer.lookup_transform(parent_frame_id, frame_id, when)
            return transform.transform
        except:
            self.logger.error(f"Failed to lookup transform between {frame_id} and {parent_frame_id}")
            return None

    def are_frames_moving(self):
        parent_transforms = []
        child_transforms = []
        start_time = time.time()
        # Collect transform over time
        def collect_tranforms(frame_id, parent_frame_id, transform_list):
            tranform = self.get_transform(frame_id, parent_frame_id)
            if tranform:
                transform_list.append(tranform)

        # Calculate relative movement
        def calculate_average_displacement():
            parent_positions = np.array([t.tranlation for t in parent_transforms])
            child_positions = np.array([t.translation for t in child_transforms])
            displacement = np.linalg.norm(parent_positions - child_positions, axis=1)
            np.mean(displacement)

        while len(parent_transforms) < 10 or len(child_transforms) < 10:  # 10 transformations
            if self.wait_for_first_transform:
                collect_tranforms(self.parent_frame_id, 'world', parent_transforms)
                collect_tranforms(self.frame_id, self.parent_frame_id, child_transforms)
                if len(parent_transforms) > 0 and len(child_transforms) > 0:
                    self.wait_for_first_transform = False
                    start_time = time.time()
            else:
                collect_tranforms(self.parent_frame_id, 'world', parent_transforms)
                collect_tranforms(self.frame_id, self.parent_frame_id, child_transforms)
                if len(parent_transforms) >= 2 or len(child_transforms) >= 2:
                    self.average_displacement = calculate_average_displacement()
                    if self.average_displacement > 0:
                        break
            if time.time() - start_time > self.timeout:
                self.logger.error(f"Timeout waiting for movement")
                return

            rclpy.spin_once(self)

        self.average_displacement = calculate_average_displacement()
