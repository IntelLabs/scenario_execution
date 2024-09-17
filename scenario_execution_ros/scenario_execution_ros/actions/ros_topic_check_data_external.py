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
import os
import importlib
from ast import literal_eval
from rosidl_runtime_py.set_message import set_message_fields
from scenario_execution_ros.actions.conversions import get_qos_preset_profile, get_ros_message_type
import builtins
from scenario_execution.actions.base_action import BaseAction, ActionError


class RosTopicCheckDataExternal(BaseAction):
    """
    Class to check if the message on ROS topic equals to the target message
    """

    def __init__(self,
                 topic_name: str,
                 topic_type: str,
                 qos_profile: tuple,
                 file_path: str,
                 function_name: str):
        super().__init__()
        self.check_function = None
        self.topic_name = topic_name
        self.topic_type = topic_type
        self.qos_profile = qos_profile
        self.file_path = file_path
        self.function_name = function_name
        self.fail_if_no_data = None
        self.fail_if_bad_comparison = None
        self.wait_for_first_message = None
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
            raise ActionError(error_message, action=self) from e

        # check if msg type exists
        try:
            get_ros_message_type(self.topic_type)()
        except (ValueError, AttributeError) as e:
            raise ActionError(f"Invalid topic type '{self.topic_type}'.", action=self) from e

        if os.path.isabs(self.file_path):
            raise ActionError(f"Only relative function_file allowed: '{self.topic_type}'.", action=self)

        if 'input_dir' not in kwargs:
            raise ActionError("input_dir not defined.", action=self)

        path = self.file_path
        if kwargs['input_dir']:
            path = os.path.join(kwargs['input_dir'], path)

        spec = importlib.util.spec_from_file_location(f"check_data_external_{self.function_name}", path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)

        try:
            self.check_function = getattr(module, self.function_name)
        except AttributeError as e:
            raise ActionError(f"Check function '{self.function_name}' not found in file '{self.file_path}'.", action=self) from e

        self.subscriber = self.node.create_subscription(
            msg_type=get_ros_message_type(self.topic_type),
            topic=self.topic_name,
            callback=self._callback,
            qos_profile=get_qos_preset_profile(self.qos_profile),
            callback_group=rclpy.callback_groups.ReentrantCallbackGroup()
        )
        self.feedback_message = f"Waiting for data on {self.topic_name}"  # pylint: disable= attribute-defined-outside-init

    def execute(self,
                fail_if_no_data: bool,
                fail_if_bad_comparison: bool,
                wait_for_first_message: bool):
        self.fail_if_no_data = fail_if_no_data
        self.fail_if_bad_comparison = fail_if_bad_comparison
        self.wait_for_first_message = wait_for_first_message
        self.found = None
        if not wait_for_first_message:
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

        self.found = self.check_function(msg)

    def set_expected_value(self, expected_value_string):
        if not isinstance(expected_value_string, str):
            raise ActionError("Only string allowed as expected_value.", action=self)
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
        except (TypeError, AttributeError) as e:
            raise ActionError(f"Could not parse '{expected_value_string}'. {error_string}", action=self) from e
