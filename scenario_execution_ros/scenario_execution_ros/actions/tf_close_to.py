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

from math import sqrt

import rclpy
from rclpy.node import Node
import py_trees
from py_trees.common import Status
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped

from tf2_ros.buffer import Buffer
from tf2_ros import TransformException  # pylint: disable= no-name-in-module

from .common import NamespacedTransformListener
from scenario_execution.actions.base_action import BaseAction, ActionError


class TfCloseTo(BaseAction):
    """
    class for distance condition
    """

    def __init__(
        self,
        associated_actor: dict,
        namespace_override: str,
        reference_point,
        threshold: float,
        sim: bool,
        robot_frame_id: str,
    ):
        super().__init__()

        if not reference_point:
            raise TypeError(f'reference_point not initialized.')
        if not threshold:
            raise TypeError(f'threshold not initialized.')

        self.namespace = associated_actor["namespace"]
        if namespace_override:
            self.namespace = namespace_override
        self.reference_point = reference_point
        self.threshold = threshold
        self.sim = sim

        if robot_frame_id:
            self.robot_frame_id = robot_frame_id
        else:
            self.robot_frame_id = 'base_link'

        self.node = None
        self.marker_handler = None
        self.marker_id = None
        self.tf_buffer = None
        self.tf_listener = None
        self.success = False

    def setup(self, **kwargs):
        """
        Setup ROS node handle and subscriber before ticking
        """
        try:
            self.node: Node = kwargs['node']
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(
                self.name, self.__class__.__name__)
            raise ActionError(error_message, action=self) from e

        try:
            self.marker_handler = kwargs['marker_handler']
        except KeyError as e:
            error_message = "didn't find 'marker_handler' in setup's kwargs [{}][{}]".format(
                self.name, self.__class__.__name__
            )
            raise ActionError(error_message, action=self) from e
        
        self.reference_point = (float(self.reference_point['x']), float(self.reference_point['y']))
        self.feedback_message = f"Waiting for transform map --> base_link"  # pylint: disable= attribute-defined-outside-init
        self.tf_buffer = Buffer()
        tf_prefix = self.namespace
        if not tf_prefix.startswith('/') and tf_prefix != '':
            tf_prefix = "/" + tf_prefix
        self.tf_listener = NamespacedTransformListener(
            node=self.node,
            buffer=self.tf_buffer,
            tf_topic=(tf_prefix + "/tf"),
            tf_static_topic=(tf_prefix + "/tf_static"),
        )

        marker = Marker()
        marker.header.frame_id = 'map'
        marker.type = Marker.CYLINDER
        marker.scale.x = self.threshold
        marker.scale.y = self.threshold
        marker.scale.z = 0.01
        marker.color.a = 1.0
        marker.color.r = 0.5
        marker.color.g = 0.5
        marker.color.b = 0.5
        marker.pose.position.x = self.reference_point[0]
        marker.pose.position.y = self.reference_point[1]
        marker.pose.position.z = 0.0
        self.marker_id = self.marker_handler.add_marker(marker)

    def update(self) -> py_trees.common.Status:
        """
        Check if the subscriber already received the right msg while ticking
        """
        translation, success = self.get_translation_from_tf()
        if not success:
            self.feedback_message = f"the pose of {self.robot_frame_id} could not be retrieved from tf"  # pylint: disable= attribute-defined-outside-init
            return Status.RUNNING
        dist = self.euclidean_dist(translation)
        marker = self.marker_handler.get_marker(self.marker_id)
        self.success = dist <= self.threshold
        if self.success:
            self.feedback_message = f"{self.robot_frame_id} reached point."  # pylint: disable= attribute-defined-outside-init
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            self.marker_handler.update_marker(self.marker_id, marker)
            return Status.SUCCESS
        else:
            self.feedback_message = f"{self.robot_frame_id} has not reached point (distance={dist-self.threshold:.2f})"  # pylint: disable= attribute-defined-outside-init
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            self.marker_handler.update_marker(self.marker_id, marker)
            return Status.RUNNING

    def get_translation_from_tf(self):
        t = None
        try:
            t = self.tf_buffer.lookup_transform('map', self.robot_frame_id, rclpy.time.Time())
            self.feedback_message = f"Transform map -> base_link got available."  # pylint: disable= attribute-defined-outside-init
        except TransformException as e:
            self.feedback_message = f"Could not transform map to base_link"  # pylint: disable= attribute-defined-outside-init
            self.node.get_logger().warn(
                f'Could not transform map to base_link: {e}')
            return None, False

        return t.transform.translation, True

    def euclidean_dist(self, pos):
        '''
        Calculate the euclidean distance between the robot position and the reference point

        Args:
            pos: Position of the robot

        return:
            Euclidean distance in float
        '''
        return sqrt((self.reference_point[0] - pos.x) ** 2 + (self.reference_point[1] - pos.y) ** 2)

    def shutdown(self):
        self.marker_handler.remove_markers([self.marker_id])
        if self.tf_listener:
            del self.tf_listener
