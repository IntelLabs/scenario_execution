# Copyright (C) 2025 Frederik Pasch
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

from ast import literal_eval
from enum import Enum
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, DurabilityPolicy
from rosidl_runtime_py.set_message import set_message_fields
import py_trees  # pylint: disable=import-error
from scenario_execution.actions.base_action import BaseAction, ActionError
from scenario_execution.model.types import VariableReference
from scenario_execution_ros.actions.conversions import get_ros_message_type, set_variable_if_available


class ServiceCallActionState(Enum):
    """
    States for executing a service call
    """
    IDLE = 1
    SERVICE_CALLED = 2
    DONE = 3
    ERROR = 4


class RosServiceCall(BaseAction):
    """
    ros service call behavior
    """

    def __init__(self, service_name: str, service_type: str, transient_local: bool = False):
        super().__init__(resolve_variable_reference_arguments_in_execute=False)
        self.node = None
        self.client = None
        self.future = None
        self.service_type_str = service_type
        self.service_type = None
        self.service_name = service_name
        self.data = None
        self.current_state = ServiceCallActionState.IDLE
        self.cb_group = ReentrantCallbackGroup()
        self.transient_local = transient_local
        self.response_variable = None
        self.response_member_name = None

    def setup(self, **kwargs):
        """
        Setup ROS2 node and service client

        """
        try:
            self.node: Node = kwargs['node']
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(
                self.name, self.__class__.__name__)
            raise ActionError(error_message, action=self) from e

        self.service_type = get_ros_message_type(self.service_type_str)

        client_kwargs = {
            "callback_group": self.cb_group,
        }

        if self.transient_local:
            qos_profile = QoSProfile(depth=1)
            qos_profile.durability = DurabilityPolicy.TRANSIENT_LOCAL
            client_kwargs["qos_profile"] = qos_profile

        self.client = self.node.create_client(
            self.service_type,
            self.service_name,
            **client_kwargs
        )

    def execute(self, data: str, response_variable: str = "", response_member_name: str = ""):
        try:
            trimmed_data = data.encode('utf-8').decode('unicode_escape')
            print(trimmed_data)
            self.data = literal_eval(trimmed_data)
        except Exception as e:  # pylint: disable=broad-except
            raise ActionError(f"Error while parsing sevice call data: {e}", action=self) from e

        if response_variable:
            if not isinstance(response_variable, VariableReference):
                raise ActionError(f"'response_variable' is expected to be a variable reference.", action=self)
            self.response_variable = response_variable
            self.response_member_name = response_member_name

        self.current_state = ServiceCallActionState.IDLE

    def update(self) -> py_trees.common.Status:
        """
        Execute states
        """
        self.logger.debug(f"Current State {self.current_state}")
        result = py_trees.common.Status.FAILURE
        if self.current_state == ServiceCallActionState.IDLE:
            self.current_state = ServiceCallActionState.SERVICE_CALLED
            if self.future:
                self.future.cancel()
            req = self.service_type.Request()
            set_message_fields(req, self.data)
            self.future = self.client.call_async(req)
            self.future.add_done_callback(self.done_callback)
            self.feedback_message = f"service request sent to {self.service_name}"  # pylint: disable= attribute-defined-outside-init
            result = py_trees.common.Status.RUNNING
        elif self.current_state == ServiceCallActionState.SERVICE_CALLED:
            self.feedback_message = f"waiting for response from {self.service_name}"  # pylint: disable= attribute-defined-outside-init
            result = py_trees.common.Status.RUNNING
        elif self.current_state == ServiceCallActionState.DONE:
            self.feedback_message = f"service response received"  # pylint: disable= attribute-defined-outside-init
            result = py_trees.common.Status.SUCCESS
        else:
            self.logger.error(f"Invalid state {self.current_state}")

        return result

    def done_callback(self, future):
        """
        Callback function when the service future is done
        """
        self.logger.debug(f"Received state {future.result()}")
        if self.current_state == ServiceCallActionState.SERVICE_CALLED:
            if self.check_response(future.result()):
                self.current_state = ServiceCallActionState.DONE
            else:
                self.current_state = ServiceCallActionState.ERROR
        else:
            self.current_state = ServiceCallActionState.ERROR

    def check_response(self, msg):
        """
        Check the result of a service call
        Can be overriden by subclasses
        """
        set_variable_if_available(msg, self.response_variable, self.response_member_name)
        return True
