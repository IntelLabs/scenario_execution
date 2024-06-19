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
from rosidl_runtime_py.set_message import set_message_fields
import py_trees  # pylint: disable=import-error
from action_msgs.msg import GoalStatus


class ActionCallActionState(Enum):
    """
    States for executing a service call
    """
    IDLE = 1
    ACTION_SERVER_AVAILABLE = 2
    ACTION_CALLED = 3
    DONE = 4
    ERROR = 5


class RosActionCall(py_trees.behaviour.Behaviour):
    """
    ros service call behavior
    """

    def __init__(self, name, action_name: str, action_type: str, data: str):
        super().__init__(name)
        self.node = None
        self.client = None
        self.send_goal_future = None
        self.goal_handle = None
        self.action_type = action_type
        self.action_name = action_name
        try:
            trimmed_data = data.encode('utf-8').decode('unicode_escape')
            self.data = literal_eval(trimmed_data)
        except Exception as e:  # pylint: disable=broad-except
            raise ValueError(f"Error while parsing sevice call data:") from e
        self.current_state = ActionCallActionState.IDLE
        self.cb_group = ReentrantCallbackGroup()

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

        datatype_in_list = self.action_type.split(".")
        try:
            self.action_type = getattr(
                importlib.import_module(".".join(datatype_in_list[0:-1])),
                datatype_in_list[-1])
        except ValueError as e:
            raise ValueError(f"Invalid action_type '{self.action_type}':") from e

        self.client = ActionClient(self.node, self.action_type, self.action_name, callback_group=self.cb_group)

    def update(self) -> py_trees.common.Status:
        """
        Execute states
        """
        self.logger.debug(f"Current State {self.current_state}")
        result = py_trees.common.Status.FAILURE
        if self.current_state == ActionCallActionState.IDLE:
            self.feedback_message = f"Waiting for action server {self.action_name}"  # pylint: disable= attribute-defined-outside-init
            if self.client.wait_for_server(0.0):
                self.current_state = ActionCallActionState.ACTION_SERVER_AVAILABLE
            result = py_trees.common.Status.RUNNING
        elif self.current_state == ActionCallActionState.ACTION_SERVER_AVAILABLE:
            self.current_state = ActionCallActionState.ACTION_CALLED
            if self.send_goal_future:
                self.send_goal_future.cancel()
            req = self.action_type.Goal()
            set_message_fields(req, self.data)
            self.send_goal_future = self.client.send_goal_async(req, feedback_callback=self.feedback_callback)
            self.send_goal_future.add_done_callback(self.goal_response_callback)
            self.feedback_message = f"action {self.action_name} called."  # pylint: disable= attribute-defined-outside-init
            result = py_trees.common.Status.RUNNING
        elif self.current_state == ActionCallActionState.ACTION_CALLED:
            result = py_trees.common.Status.RUNNING
        elif self.current_state == ActionCallActionState.DONE:
            result = py_trees.common.Status.SUCCESS
        else:
            self.logger.error(f"Invalid state {self.current_state}")

        return result

    def feedback_callback(self, msg):
        if self.current_state not in [ActionCallActionState.DONE, ActionCallActionState.ERROR]:
            print(msg.feedback)
            self.feedback_message = f"Current: {msg.feedback}"  # pylint: disable= attribute-defined-outside-init

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
                self.feedback_message = f"action successfully finished."  # pylint: disable= attribute-defined-outside-init
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
