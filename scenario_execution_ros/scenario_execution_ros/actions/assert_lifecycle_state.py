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

import py_trees  # pylint: disable=import-error
from rclpy.node import Node
from lifecycle_msgs.srv import GetState


class AssertLifecycleState(py_trees.behaviour.Behaviour):

    def __init__(self, name, node_name: str, state: str, fail_on_finish: bool):
        super().__init__(name)
        self.node_name = node_name
        self.node_state = state
        self.fail_on_finish = fail_on_finish
        self.service_called = False
        self.client = None
        self.actual_state = None
        self.node = None
        self.subscription = None

    def setup(self, **kwargs):
        try:
            self.node: Node = kwargs['node']
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(
                self.name, self.__class__.__name__)
            raise KeyError(error_message) from e

        service_get_state_name = "/" + self.node_name + "/get_state"
        self.client = self.node.create_client(GetState, service_get_state_name)

    def update(self) -> py_trees.common.Status:
        result = py_trees.common.Status.FAILURE
        service_check = self.check_service_ready()
        if service_check and not self.service_called:
            self.send_request()
            self.service_called = True
            result = py_trees.common.Status.RUNNING
        elif service_check and self.service_called:
            if self.actual_state:
                if self.actual_state == self.node_state:
                    self.feedback_message = f"The node '{self.node_name}' is in state '{self.actual_state}'."  # pylint: disable= attribute-defined-outside-init
                    result = py_trees.common.Status.RUNNING
                elif self.fail_on_finish:
                    self.feedback_message = f"Node '{self.node_name}' state '{self.actual_state}' doesn't match '{self.node_state}'."  # pylint: disable= attribute-defined-outside-init
                    result = py_trees.common.Status.FAILURE
                else:
                    self.feedback_message = f"Node '{self.node_name}' state '{self.actual_state}' doesn't match '{self.node_state}'."  # pylint: disable= attribute-defined-outside-init
                    result = py_trees.common.Status.SUCCESS
            else:
                self.feedback_message = f"Failed to get lifecycle state for node {self.node_name}."  # pylint: disable= attribute-defined-outside-init
                result = py_trees.common.Status.FAILURE
        else:
            self.feedback_message = f"Service not available, waiting again..."  # pylint: disable= attribute-defined-outside-init
            result = py_trees.common.Status.RUNNING
        return result

    def check_service_ready(self):
        is_service = self.client.wait_for_service(timeout_sec=1.0)
        return is_service

    def send_request(self):
        req = GetState.Request()
        future = self.client.call_async(req)
        future.add_done_callback(self.handle_state_response)

    def handle_state_response(self, future):
        try:
            response = future.result()
            if response:
                self.actual_state = response.current_state.label
            else:
                self.logger.error("Failed to get response.")
        except Exception as e:  # pylint: disable=broad-except
            self.logger.error(f"Exception in getting lifecycle state: str({e})")
