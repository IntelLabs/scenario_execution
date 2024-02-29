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

""" Scenario execution plugin to put ros data into the blackboard """
from ast import literal_eval
import importlib
from enum import Enum
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rosidl_runtime_py.set_message import set_message_fields
import py_trees  # pylint: disable=import-error


class ServiceCallActionState(Enum):
    """
    States for executing a service call
    """
    IDLE = 1
    SERVICE_CALLED = 2
    DONE = 3
    ERROR = 4


class RosServiceCall(py_trees.behaviour.Behaviour):
    """
    ros service call behavior

    Args:
        service_name: name of the topic to connect to
        service_type: The service type
        call: call content
    """

    def __init__(self, name, service_name: str, service_type: str, data: str):
        super().__init__(name)
        self.node = None
        self.client = None
        self.future = None
        self.service_type = service_type
        self.service_name = service_name
        try:
            trimmed_data = data.encode('utf-8').decode('unicode_escape')
            self.data = literal_eval(trimmed_data)
        except Exception as e:  # pylint: disable=broad-except
            raise ValueError(f"Error while parsing sevice call data:") from e
        self.current_state = ServiceCallActionState.IDLE
        self.cb_group = ReentrantCallbackGroup()

    def setup(self, **kwargs):
        """
        Setup ROS2 node and service client

        """
        try:
            self.node: Node = kwargs['node']
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(
                self.name, self.__class__.__name__)
            raise KeyError(error_message) from e  # 'direct cause' traceability

        datatype_in_list = self.service_type.split(".")
        self.service_type = getattr(
            importlib.import_module(".".join(datatype_in_list[0:-1])),
            datatype_in_list[-1])

        self.client = self.node.create_client(
            self.service_type, self.service_name, callback_group=self.cb_group)

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

    def check_response(self, _):
        """
        Check the result of a service call
        Can be overriden by subclasses
        """
        return True
