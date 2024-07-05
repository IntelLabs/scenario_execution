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
from scenario_execution.actions.base_action import BaseAction
from enum import Enum
from queue import Queue
from collections import deque


class AssertLifecycleStateState(Enum):
    IDLE = 1
    WAITING_FOR_SERVICE = 2
    WAITING_FOR_INITIAL_STATE = 3
    CHECKING_STATE = 4
    DONE = 5
    FAILURE = 6


class AssertLifecycleState(BaseAction):

    def __init__(self, node_name: str, state_sequence: list, allow_initial_skip: bool, fail_on_unexpected: bool, keep_running: bool):
        super().__init__()
        self.current_state = AssertLifecycleStateState.IDLE
        self.node_name = node_name
        self.state_sequence = state_sequence
        self.allow_initial_skip = allow_initial_skip
        self.fail_on_unexpected = fail_on_unexpected
        self.keep_running = keep_running
        self.node = None
        self.subscription = None
        self.initial_states_skipped = False
        self.client = None
        self.received_state_transition_queue = Queue()

    def setup(self, **kwargs):
        try:
            self.node: Node = kwargs['node']
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(
                self.name, self.__class__.__name__)
            raise KeyError(error_message) from e

        topic_transition_event_name = "/" + self.node_name + "/transition_event"
        self.subscription = self.node.create_subscription(
            TransitionEvent, topic_transition_event_name, self.lifecycle_callback, qos_profile=get_qos_preset_profile(['system_default']))

        service_get_state_name = "/" + self.node_name + "/get_state"
        self.client = self.node.create_client(GetState, service_get_state_name)

    def execute(self, node_name: str, state_sequence: list, allow_initial_skip: bool, fail_on_unexpected: bool, keep_running: bool):
        if self.node_name != node_name or self.state_sequence != state_sequence:
            raise ValueError("Updating node name or state_sequence not supported.")

        if all(isinstance(state, tuple) and len(state) == 2 for state in self.state_sequence):
            self.state_sequence = [state[0] for state in self.state_sequence]
        else:
            allowed_states = ['unconfigured', 'inactive', 'active', 'finalized']
            for value in self.state_sequence:
                if value not in allowed_states:
                    raise ValueError("The specified state_sequence is not valid")
        self.state_sequence = deque(self.state_sequence)
        self.allow_initial_skip = allow_initial_skip
        self.fail_on_unexpected = fail_on_unexpected
        self.keep_running = keep_running

    def update(self) -> py_trees.common.Status:
        if self.current_state == AssertLifecycleStateState.IDLE:
            if self.client.service_is_ready():
                self.get_initial_state()
                self.current_state = AssertLifecycleStateState.WAITING_FOR_INITIAL_STATE
            else:
                self.feedback_message = f"Waiting for service..."  # pylint: disable= attribute-defined-outside-init
            return py_trees.common.Status.RUNNING
        elif self.current_state == AssertLifecycleStateState.WAITING_FOR_INITIAL_STATE:
            return py_trees.common.Status.RUNNING
        elif self.current_state == AssertLifecycleStateState.CHECKING_STATE:
            while not self.received_state_transition_queue.empty():
                state = self.received_state_transition_queue.get()
                if not self.state_sequence:
                    self.feedback_message = f"Unexpected state '{state}' reached..."  # pylint: disable= attribute-defined-outside-init
                    if self.fail_on_unexpected:
                        return py_trees.common.Status.FAILURE
                    else:
                        return py_trees.common.Status.SUCCESS
                if self.state_sequence[0] == state:
                    self.state_sequence.popleft()
                else:
                    if self.allow_initial_skip and not self.initial_states_skipped:
                        found = False
                        skipped = []
                        while not found and self.state_sequence:
                            elem = self.state_sequence.popleft()
                            skipped.append(elem)
                            found = elem == state
                        self.feedback_message = f"Skipped {skipped} states."  # pylint: disable= attribute-defined-outside-init
                        self.initial_states_skipped = True
                    else:
                        return py_trees.common.Status.FAILURE
            if not self.state_sequence:
                if self.keep_running:
                    self.feedback_message = f"Final state reached. Continue monitoring..."  # pylint: disable= attribute-defined-outside-init
                    return py_trees.common.Status.RUNNING
                else:
                    return py_trees.common.Status.SUCCESS
            return py_trees.common.Status.RUNNING

        return py_trees.common.Status.FAILURE

    def get_initial_state(self):
        self.feedback_message = "Getting initial lifecycle state..."  # pylint: disable= attribute-defined-outside-init
        req = GetState.Request()
        future = self.client.call_async(req)
        future.add_done_callback(self.handle_initial_state_response)

    def add_to_queue(self, elem):
        if elem not in ('configuring', 'cleaningup', 'deactivating', 'activating'):
            self.received_state_transition_queue.put(elem)

    def handle_initial_state_response(self, future):
        if self.current_state != AssertLifecycleStateState.WAITING_FOR_INITIAL_STATE:
            return
        try:
            response = future.result()
            if response:
                self.add_to_queue(response.current_state.label)
                self.current_state = AssertLifecycleStateState.CHECKING_STATE
                # self.feedback_message = f"State '{response.current_state.label}' reached."  # pylint: disable= attribute-defined-outside-init
            else:
                self.logger.error("Failed to get initial state.")
                self.current_state = AssertLifecycleStateState.FAILURE
        except Exception as e:  # pylint: disable=broad-except
            self.current_state = AssertLifecycleStateState.FAILURE
            self.logger.error(f"Exception in getting inital state: str({e})")

    def lifecycle_callback(self, msg):
        if self.current_state == AssertLifecycleStateState.WAITING_FOR_INITIAL_STATE:
            self.current_state = AssertLifecycleStateState.CHECKING_STATE
        self.add_to_queue(msg.goal_state.label)
