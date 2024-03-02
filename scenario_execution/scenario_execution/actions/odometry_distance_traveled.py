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

""" Scenario execution plugin for waiting for a certain covered distance, based on odometry """
from math import sqrt
import rclpy
from rclpy.logging import get_logger
from nav_msgs.msg import Odometry
from py_trees.common import Status
import py_trees


class OdometryDistanceTraveled(py_trees.behaviour.Behaviour):
    """
    Class to wait for a certain covered distance, based on odometry
    """

    def __init__(self, name, associated_actor, distance: float):
        super().__init__(name)
        self.namespace = associated_actor["namespace"]
        self.distance_expected = distance
        self.distance_traveled = 0.
        self.previous_x = 0
        self.previous_y = 0
        self.first_run = True
        self.logger = None
        self.node = None
        self.subscriber = None
        self.callback_group = None

    def setup(self, **kwargs):
        """
        Setup subscription and logger
        """
        self.logger = get_logger('odometry_distance_traveled')
        self.logger.debug(f"Waiting for traveled distance of {self.distance_expected}")

        try:
            self.node = kwargs['node']
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(
                self.name, self.__class__.__name__)
            raise KeyError(error_message) from e  # 'direct cause' traceability
        self.callback_group = rclpy.callback_groups.MutuallyExclusiveCallbackGroup()
        self.subscriber = self.node.create_subscription(
            Odometry, self.namespace + '/odom', self._callback, 1000, callback_group=self.callback_group)

    def _callback(self, msg):
        '''
        Subscriber callback
        '''
        self.calculate_distance(msg)

    def calculate_distance(self, msg):
        """
        Update the odometry distance
        Args:
            msg [Odometry]: current odometry message to update
        """
        if self.first_run:
            self.first_run = False
        else:
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            d_increment = sqrt((x - self.previous_x) * (x - self.previous_x) +
                               (y - self.previous_y) * (y - self.previous_y))
            self.distance_traveled = self.distance_traveled + d_increment
            self.logger.debug(f'Total distance traveled is {self.distance_traveled}m')

        self.previous_x = msg.pose.pose.position.x
        self.previous_y = msg.pose.pose.position.y

    def initialise(self):
        '''
        Initialize before ticking.
        '''
        self.distance_traveled = 0.
        self.previous_x = 0
        self.previous_y = 0
        self.first_run = True

    def update(self) -> py_trees.common.Status:
        """
        Check if the traveled distance is reached
        return:
            py_trees.common.Status.SUCCESS if the distanced is reached, else
            return py_trees.common.Status.RUNNING.
        """

        self.logger.debug(f"ticking: {self.distance_traveled}")
        if self.distance_traveled >= self.distance_expected:
            self.feedback_message = f"expected traveled distance reached: {self.distance_expected:.3}"  # pylint: disable= attribute-defined-outside-init
            return Status.SUCCESS
        else:
            self.feedback_message = f"distance traveled: {self.distance_traveled:.3} < {self.distance_expected:.3}"  # pylint: disable= attribute-defined-outside-init
        return Status.RUNNING
