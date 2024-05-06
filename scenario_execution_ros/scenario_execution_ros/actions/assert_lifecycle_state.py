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
from lifecycle_msgs.msg import TransitionEvent
from scenario_execution_ros.actions.conversions import get_qos_preset_profile


class AssertLifecycleState(py_trees.behaviour.Behaviour):

    def __init__(self, name, node_name: str, state_sequence: list, allow_inital_state_skip: bool, fail_on_finish: bool):
        super().__init__(name)
        self.node_name = node_name
        self.state_sequence = state_sequence
        self.allow_inital_state_skip = allow_inital_state_skip
        self.fail_on_finish = fail_on_finish
        self.node = None
        self.subscription = None
        self.current_index = 0  # Start with the first expected state
        self.current_state = None
        self.expected_state = None
        self.service_check = False
        self.client = None
        self.valid_transitions = {
            'unconfigured': ['configuring', 'cleaningup'],  # Valid intermediate state for each expected state
            'inactive': ['configuring', 'deactivating'],
            'active': ['activating'],
            'finalized': []
        }

    def setup(self, **kwargs):
        try:
            self.node: Node = kwargs['node']
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(
                self.name, self.__class__.__name__)
            raise KeyError(error_message) from e

        if all(isinstance(state, tuple) and len(state) == 2 for state in self.state_sequence):
            self.state_sequence = [state[0] for state in self.state_sequence]
        else:
            allowed_states = ['unconfigured', 'inactive', 'active', 'finalized']
            for value in self.state_sequence:
                if value not in allowed_states:
                    raise ValueError("The specified state_sequence is not valid")

        service_get_state_name = "/" + self.node_name + "/get_state"
        self.client = self.node.create_client(GetState, service_get_state_name)

    def update(self) -> py_trees.common.Status:
        if not self.service_check:
            self.check_service_ready()
            self.feedback_message = "Service not available, waiting..."  # pylint: disable= attribute-defined-outside-init
            return py_trees.common.Status.RUNNING
        if self.current_state and self.expected_state:
            if self.current_state in self.valid_transitions.get(self.expected_state, []):
                self.feedback_message = f'{self.node_name}: Transitioning through valid state {self.current_state}.'  # pylint: disable= attribute-defined-outside-init
                return py_trees.common.Status.RUNNING
            elif self.current_state == self.expected_state:
                self.feedback_message = f"{self.node_name}: Currently in state {self.current_state}."  # pylint: disable= attribute-defined-outside-init
                return py_trees.common.Status.RUNNING
            else:
                self.feedback_message = f'{self.node_name}: Unexpected state transition. Expected {self.expected_state} or valid intermediate, but got {self.current_state}.'  # pylint: disable= attribute-defined-outside-init
                return py_trees.common.Status.FAILURE if self.fail_on_finish else py_trees.common.Status.SUCCESS
        else:
            self.feedback_message = f"{self.node_name}: Waiting for current state..."  # pylint: disable= attribute-defined-outside-init
            return py_trees.common.Status.RUNNING

    def check_service_ready(self):
        is_service = self.client
        if is_service:
            topic_transition_event_name = "/" + self.node_name + "/transition_event"
            self.subscription = self.node.create_subscription(
                TransitionEvent, topic_transition_event_name, self.lifecycle_callback, qos_profile=get_qos_preset_profile(['sensor_data']))
            self.get_initial_state()
            self.service_check = True
        else:
            self.service_check = False

    def get_initial_state(self):
        req = GetState.Request()
        future = self.client.call_async(req)
        future.add_done_callback(self.handle_initial_state_response)

    def handle_initial_state_response(self, future):
        try:
            response = future.result()
            if response:
                self.current_state = response.current_state.label
                if self.allow_inital_state_skip and self.current_state in self.state_sequence:
                    self.current_index = self.state_sequence.index(self.current_state)
                if self.current_index < len(self.state_sequence):
                    self.expected_state = self.state_sequence[self.current_index]
                    self.current_index += 1
            else:
                self.logger.error("Failed to get initial state.")
        except Exception as e:  # pylint: disable=broad-except
            self.logger.error(f"Exception in getting inital state: str({e})")

    def lifecycle_callback(self, msg):
        goal_label = msg.goal_state.label
        if goal_label:
            self.current_state = goal_label
            if self.current_index < len(self.state_sequence):
                self.expected_state = self.state_sequence[self.current_index]
                if self.expected_state == self.current_state:
                    self.current_index += 1
            else:
                self.current_index = len(self.state_sequence)
