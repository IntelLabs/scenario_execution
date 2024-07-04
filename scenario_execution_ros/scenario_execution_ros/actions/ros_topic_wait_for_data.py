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

from scenario_execution_ros.actions.conversions import get_qos_preset_profile, get_ros_message_type
from scenario_execution.actions.base_action import BaseAction
import rclpy
import py_trees


class RosTopicWaitForData(BaseAction):
    """
    Class to check if the message on ROS topic equals to the target message

    Args:
        topic_name[str]: name of the topic to connect to
        topic_type[str]: class of the message type (e.g. std_msgs.msg.String)
        qos_profile[str]: qos profile for the subscriber
    """

    def __init__(self, topic_name: str, topic_type: str, qos_profile: tuple):
        super().__init__()
        self.topic_type = topic_type
        self.qos_profile = qos_profile
        self.topic_name = topic_name
        self.subscriber = None
        self.node = None
        self.found = None

    def setup(self, **kwargs):
        """
        Setup the subscriber
        """
        try:
            self.node: rclpy.Node = kwargs['node']
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(
                self.name, self.__class__.__name__)
            raise KeyError(error_message) from e

        self.subscriber = self.node.create_subscription(
            msg_type=get_ros_message_type(self.topic_type),
            topic=self.topic_name,
            callback=self._callback,
            qos_profile=get_qos_preset_profile(self.qos_profile),
            callback_group=rclpy.callback_groups.ReentrantCallbackGroup()
        )
        self.feedback_message = f"Waiting for data on {self.topic_name}"  # pylint: disable= attribute-defined-outside-init

    def execute(self, topic_name, topic_type, qos_profile):
        if self.topic_name != topic_name or self.topic_type != topic_type or self.qos_profile != qos_profile:
            raise ValueError("Updating topic parameters not supported.")
        self.found = False

    def update(self) -> py_trees.common.Status:
        if self.found is True:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.RUNNING

    def _callback(self, msg):
        if self.found is None:  # do not check until action gets executed
            return
        self.found = True
        self.feedback_message = f"Received data on {self.topic_name}"  # pylint: disable= attribute-defined-outside-init
