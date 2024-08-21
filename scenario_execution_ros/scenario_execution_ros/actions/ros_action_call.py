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
import importlib
from enum import Enum
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy
from rosidl_runtime_py.set_message import set_message_fields
import py_trees  # pylint: disable=import-error
from action_msgs.msg import GoalStatus
from scenario_execution.actions.base_action import BaseAction


class ActionCallActionState(Enum):
    """
    States for executing a service call
    """
    IDLE = 1
    ACTION_SERVER_AVAILABLE = 2
    ACTION_CALLED = 3
    DONE = 4
    ERROR = 5


class RosActionCall(BaseAction):
    """
    ros service call behavior
    """

    def __init__(self, action_name: str, action_type: str, data: str, transient_local: bool = False):
        super().__init__()
        self.node = None
        self.client = None
        self.send_goal_future = None
        self.goal_handle = None
        self.action_type_string = action_type
        self.action_type = None
        self.action_name = action_name
        self.received_feedback = None
        self.data = None
        self.parse_data(data)
        self.current_state = ActionCallActionState.IDLE
        self.cb_group = ReentrantCallbackGroup()
        self.transient_local = transient_local
        
    def setup(self, **kwargs):
        """
        Setup ROS2 node and action client

        """
        try:
            self.node: Node = kwargs['node']
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(
                self.name, self.__class__.__name__)
            raise KeyError(error_message) from e

        datatype_in_list = self.action_type_string.split(".")
        try:
            self.action_type = getattr(
                importlib.import_module(".".join(datatype_in_list[0:-1])),
                datatype_in_list[-1])
        except ValueError as e:
            raise ValueError(f"Invalid action_type '{self.action_type}':") from e

        client_kwargs = {
            "callback_group": self.cb_group,
        }

        if self.transient_local:
            qos_profile = QoSProfile(depth=1)
            qos_profile.durability = DurabilityPolicy.TRANSIENT_LOCAL
            client_kwargs["result_service_qos_profile"] = qos_profile

        self.client = ActionClient(self.node, self.action_type, self.action_name, **client_kwargs)

    def execute(self, action_name: str, action_type: str, data: str, transient_local: bool):
        if self.action_name != action_name or self.action_type_string != action_type or self.transient_local != transient_local:
            raise ValueError(f"Updating action_name or action_type_string not supported.")

        self.parse_data(data)
        self.current_state = ActionCallActionState.IDLE

    def parse_data(self, data):
        if data:
            try:
                trimmed_data = data.encode('utf-8').decode('unicode_escape')
                self.data = literal_eval(trimmed_data)
            except Exception as e:  # pylint: disable=broad-except
                raise ValueError(f"Error while parsing sevice call data:") from e

    def update(self) -> py_trees.common.Status:
        """
        Execute states
        """
        self.logger.debug(f"Current State {self.current_state}")
        result = py_trees.common.Status.FAILURE
        if self.current_state == ActionCallActionState.IDLE:
            if self.client.wait_for_server(0.0):
                self.current_state = ActionCallActionState.ACTION_SERVER_AVAILABLE
            result = py_trees.common.Status.RUNNING
        elif self.current_state == ActionCallActionState.ACTION_SERVER_AVAILABLE:
            self.current_state = ActionCallActionState.ACTION_CALLED
            if self.send_goal_future:
                self.send_goal_future.cancel()
            self.send_goal_future = self.client.send_goal_async(self.get_goal_msg(), feedback_callback=self.feedback_callback)
            self.send_goal_future.add_done_callback(self.goal_response_callback)
            result = py_trees.common.Status.RUNNING
        elif self.current_state == ActionCallActionState.ACTION_CALLED:
            result = py_trees.common.Status.RUNNING
        elif self.current_state == ActionCallActionState.DONE:
            result = py_trees.common.Status.SUCCESS
        else:
            self.logger.error(f"Invalid state {self.current_state}")
        feedback_msg = self.get_feedback_message(self.current_state)
        if feedback_msg is not None:
            self.feedback_message = feedback_msg  # pylint: disable= attribute-defined-outside-init

        return result

    def get_goal_msg(self):
        req = self.action_type.Goal()
        set_message_fields(req, self.data)
        return req

    def feedback_callback(self, msg):
        self.received_feedback = msg.feedback

    def goal_response_callback(self, future):
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.feedback_message = f"Goal rejected."  # pylint: disable= attribute-defined-outside-init
            self.current_state = ActionCallActionState.ERROR
            return
        self.feedback_message = f"Goal accepted."  # pylint: disable= attribute-defined-outside-init
        get_result_future = self.goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """
        Callback function when the action future is done
        """
        status = future.result().status
        self.logger.debug(f"Received state {status}")
        if self.current_state == ActionCallActionState.ACTION_CALLED:
            if status == GoalStatus.STATUS_SUCCEEDED:
                self.current_state = ActionCallActionState.DONE
                self.goal_handle = None
            elif status == GoalStatus.STATUS_CANCELED:
                self.current_state = ActionCallActionState.ERROR
                self.feedback_message = f"Goal canceled."   # pylint: disable= attribute-defined-outside-init
                self.goal_handle = None
            elif status == GoalStatus.STATUS_ABORTED:
                self.current_state = ActionCallActionState.ERROR
                self.feedback_message = f"Goal aborted."   # pylint: disable= attribute-defined-outside-init
                self.goal_handle = None
        else:
            self.current_state = ActionCallActionState.ERROR

    def shutdown(self):
        if self.goal_handle:
            self.goal_handle.cancel_goal()

    def get_feedback_message(self, current_state):
        feedback_message = None
        if current_state == ActionCallActionState.IDLE:
            feedback_message = f"Waiting for action server {self.action_name}"
        elif current_state == ActionCallActionState.ACTION_CALLED:
            if self.received_feedback is not None:
                feedback_message = f"Current: {self.received_feedback}"
            else:
                feedback_message = f"Action {self.action_name} called."
        elif current_state == ActionCallActionState.DONE:
            feedback_message = f"Action successfully finished."
        return feedback_message
