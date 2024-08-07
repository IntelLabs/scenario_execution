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
import operator
from ast import literal_eval
from rosidl_runtime_py.set_message import set_message_fields
from scenario_execution_ros.actions.conversions import get_qos_preset_profile, get_comparison_operator, get_ros_message_type
import builtins
from scenario_execution.actions.base_action import BaseAction


class RosTopicCheckData(BaseAction):
    """
    Class to check if the message on ROS topic equals to the target message
    """

    def __init__(self,
                 topic_name: str,
                 topic_type: str,
                 qos_profile: tuple,
                 member_name: str,
                 expected_value: str,
                 comparison_operator: int,
                 fail_if_no_data: bool,
                 fail_if_bad_comparison: bool,
                 wait_for_first_message: bool):
        super().__init__()
        self.topic_name = topic_name
        self.topic_type = topic_type
        self.qos_profile = qos_profile
        self.member_name = member_name
        self.set_expected_value(expected_value)
        self.comparison_operator = get_comparison_operator(comparison_operator)
        self.fail_if_no_data = fail_if_no_data
        self.fail_if_bad_comparison = fail_if_bad_comparison
        self.wait_for_first_message = wait_for_first_message
        self.last_msg = None
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

    def execute(self,
                topic_name: str,
                topic_type: str,
                qos_profile: tuple,
                member_name: str,
                expected_value: str,
                comparison_operator: int,
                fail_if_no_data: bool,
                fail_if_bad_comparison: bool,
                wait_for_first_message: bool):
        if self.topic_name != topic_name or self.topic_type != topic_type or self.qos_profile != qos_profile:
            raise ValueError("Updating topic parameters not supported.")
        self.member_name = member_name
        self.set_expected_value(expected_value)
        self.comparison_operator = get_comparison_operator(comparison_operator)
        self.fail_if_no_data = fail_if_no_data
        self.fail_if_bad_comparison = fail_if_bad_comparison
        self.wait_for_first_message = wait_for_first_message
        if wait_for_first_message:
            self.found = False
        else:
            self.check_data(self.last_msg)
            if self.found is True:
                self.feedback_message = f"Found expected value in previously received message."  # pylint: disable= attribute-defined-outside-init

    def update(self) -> py_trees.common.Status:
        if self.found is True:
            return py_trees.common.Status.SUCCESS
        elif self.found is False:
            if self.fail_if_bad_comparison:
                return py_trees.common.Status.FAILURE

        return py_trees.common.Status.RUNNING

    def _callback(self, msg):
        self.last_msg = msg
        self.check_data(msg)
        if self.found is True:
            self.feedback_message = f"Found expected value in received message."
        else:
            self.feedback_message = f"Received message does not contain expected value."

    def check_data(self, msg):
        if msg is None:
            return

        if self.member_name == "":
            value = msg
        else:
            check_attr = operator.attrgetter(self.member_name)
            try:
                value = check_attr(msg)
            except AttributeError:
                self.feedback_message = "Member name not found {self.member_name}]"
        self.found = self.comparison_operator(value, self.expected_value)

    def set_expected_value(self, expected_value_string):
        if not isinstance(expected_value_string, str):
            raise ValueError("Only string allowed as expected_value.")
        error_string = ""
        try:
            parsed_value = literal_eval("".join(expected_value_string.split('\\')))
            msg = get_ros_message_type(self.topic_type)()
            if self.member_name == "":
                self.expected_value = msg
                set_message_fields(self.expected_value, parsed_value)
            else:
                self.expected_value = getattr(msg, self.member_name)
                error_string = f"Member {self.member_name} is expected to be of type {type(self.expected_value).__name__}. "
                if type(self.expected_value).__name__ in dir(builtins):
                    self.expected_value = parsed_value
                else:
                    set_message_fields(self.expected_value, parsed_value)
        except TypeError as e:
            raise ValueError(f"Could not parse '{expected_value_string}'. {error_string}") from e
