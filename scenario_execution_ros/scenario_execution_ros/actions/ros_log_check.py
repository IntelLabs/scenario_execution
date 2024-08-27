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
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy
import rclpy
from rcl_interfaces.msg import Log
from py_trees.common import Status
from scenario_execution.actions.base_action import BaseAction, ActionError


class RosLogCheck(BaseAction):
    """
    Class for scanning the ros log for specific content
    """

    def __init__(self, values: list, module_name: str):
        super().__init__()
        if not isinstance(values, list):
            raise TypeError(f'Value needs to be list of strings, got {type(values)}.')
        else:
            self.values = values

        self.subscriber = None
        self.node = None
        self.found = None
        self.module_name = module_name

    def setup(self, **kwargs):
        """
        Setup the subscriber
        """
        try:
            self.node: rclpy.Node = kwargs['node']
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(
                self.name, self.__class__.__name__)
            raise ActionError(error_message, action=self) from e

        topic_qos = QoSProfile(
            depth=100,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST)

        self.subscriber = self.node.create_subscription(  # pylint: disable= attribute-defined-outside-init
            msg_type=Log,
            topic='/rosout',
            callback=self._callback,
            qos_profile=topic_qos,
            callback_group=rclpy.callback_groups.ReentrantCallbackGroup()
        )
        self.feedback_message = f"Waiting for log"  # pylint: disable= attribute-defined-outside-init

    def execute(self, values: list, module_name: str):
        self.module_name = module_name
        self.values = values
        self.found = None

    def update(self) -> py_trees.common.Status:
        """
        Wait for specified log entries

        return:
            py_trees.common.Status if found
        """
        if self.found is None:
            self.found = False  # first update() tick, start checking logs

        if self.found:
            return Status.SUCCESS
        else:
            return Status.RUNNING

    def _callback(self, msg):
        if self.found is None:
            return

        if msg.name == self.node.get_name():  # skip own logs
            return

        if self.module_name and self.module_name != msg.name:
            return

        for val in self.values:
            if val in msg.msg:
                self.feedback_message = f"Found string '{val}' in '{msg}'"  # pylint: disable= attribute-defined-outside-init
                self.found = True
                break
