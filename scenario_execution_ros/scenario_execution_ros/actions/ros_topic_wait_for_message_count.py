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

import py_trees
import rclpy
from py_trees.common import Status
from scenario_execution_ros.actions.conversions import get_qos_preset_profile
import importlib


class RosWaitForMessageCount(py_trees.behaviour.Behaviour):
    """
    Class for waiting until a certain amount of messages was received
    """

    def __init__(self, name, topic_name: str, topic_type: str, qos_profile, count: int):
        super().__init__(name)

        self.subscriber = None
        self.node = None
        self.topic_name = topic_name
        self.expected_count = count
        self.current_count = 0
        self.last_count_reported = 0

        datatype_in_list = topic_type.split(".")
        self.topic_type = getattr(
            importlib.import_module(".".join(datatype_in_list[0:-1])),
            datatype_in_list[-1]
        )

        self.qos_profile = get_qos_preset_profile(qos_profile)
        self.qos_profile.depth=self.expected_count

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

        self.subscriber = self.node.create_subscription(  # pylint: disable= attribute-defined-outside-init
            msg_type=self.topic_type,
            topic=self.topic_name,
            callback=self._callback,
            qos_profile=self.qos_profile,
            callback_group=rclpy.callback_groups.MutuallyExclusiveCallbackGroup()
        )
        self.feedback_message = f"Waiting for messages on {self.topic_name}"  # pylint: disable= attribute-defined-outside-init

    def update(self) -> py_trees.common.Status:
        if self.current_count >= self.expected_count:
            self.feedback_message = f"Received expected ({self.expected_count}) messages."  # pylint: disable= attribute-defined-outside-init
            return Status.SUCCESS
        else:
            if self.last_count_reported != self.current_count:
                self.feedback_message = f"Received {self.current_count} of expected {self.expected_count}..."  # pylint: disable= attribute-defined-outside-init
                self.last_count_reported = self.current_count
            return Status.RUNNING

    def _callback(self, _):
        self.current_count += 1
