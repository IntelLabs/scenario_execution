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
from scenario_execution.actions.base_action import BaseAction, ActionError
from scenario_execution.model.types import VariableReference
import rclpy
import py_trees
import operator


class RosTopicMonitor(BaseAction):

    def __init__(self, topic_name: str, topic_type: str, qos_profile: tuple):
        super().__init__(resolve_variable_reference_arguments_in_execute=False)
        self.target_variable = None
        self.member_name = None
        self.topic_type = topic_type
        self.qos_profile = qos_profile
        self.topic_name = topic_name
        self.subscriber = None
        self.node = None

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

        msg_type = get_ros_message_type(self.topic_type)

        # check if member-name exists
        self.get_value(msg_type())

        self.subscriber = self.node.create_subscription(
            msg_type=msg_type,
            topic=self.topic_name,
            callback=self._callback,
            qos_profile=get_qos_preset_profile(self.qos_profile),
            callback_group=rclpy.callback_groups.ReentrantCallbackGroup()
        )
        self.feedback_message = f"Monitoring data on {self.topic_name}"  # pylint: disable= attribute-defined-outside-init

    def execute(self, member_name: str, target_variable: object):
        if not isinstance(target_variable, VariableReference):
            raise ActionError(f"'target_variable' is expected to be a variable reference.", action=self)
        self.target_variable = target_variable
        self.member_name = member_name

    def update(self) -> py_trees.common.Status:
        return py_trees.common.Status.SUCCESS

    def _callback(self, msg):
        if self.target_variable is not None:
            self.target_variable.set_value(self.get_value(msg))

    def get_value(self, msg):
        if self.member_name is not None and self.member_name != "":
            check_attr = operator.attrgetter(self.member_name)
            try:
                return check_attr(msg)
            except AttributeError as e:
                raise ActionError(f"invalid member_name '{self.member_name}'", action=self) from e
        else:
            return msg
