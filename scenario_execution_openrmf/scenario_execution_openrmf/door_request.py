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

""" Plugin to manage doors """

from enum import Enum
from rclpy.node import Node

import py_trees
from rmf_door_msgs.msg import DoorRequest as DoorRequestMsg
from rmf_door_msgs.msg import DoorState
import time


class DoorRequestState(Enum):
    IDLE = 1
    DOOR_MODE_REQUESTED = 3
    DONE = 3
    FAILURE = 4


class DoorRequest(py_trees.behaviour.Behaviour):

    def __init__(self, name, door_ids: str, door_mode):
        super().__init__(name)
        self.door_ids = door_ids
        self.requested_doors = []
        self.requested_doors_list = ""
        self.retrieve_doors = []
        self.requested_door_mode_value = door_mode[1]
        self.requested_door_mode_state = door_mode[0]
        self.doors_current_state = {}
        self.current_state = DoorRequestState.IDLE
        self.all_doors_executed = False
        self.publisher = None
        self.subscriber = None
        self.feedback_message = None
        self.node = None
        self.future = None
        self.result_future = None
        self.goal_handle = None
        self.feedback = None
        self.request_result = None
        self.status = None

    def setup(self, **kwargs):
        try:
            self.node: Node = kwargs['node']
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(
                self.name, self.__class__.__name__)
            raise KeyError(error_message) from e

        self.publisher = self.node.create_publisher(DoorRequestMsg, "door_requests", 10)

        timeout_sec = 60
        start_time = time.time()
        while not self.publisher.get_subscription_count():
            if time.time() - start_time > timeout_sec:
                raise KeyError("Timeout waiting for topic door_requests!")
            self.logger.error('No doors available. Retrying...')
            time.sleep(2)

        self.subscriber = self.node.create_subscription(DoorState, "door_states", self.callback_get_door_states, 10)

        # get the doors when door_id is passed as an empty string
        if not self.door_ids:
            self.feedback_message = 'Retrieving Doors from Simulation....'
            self.requested_doors = self.retrieve_doors
        else:
            self.requested_doors = [s.strip() for s in self.door_ids.split(' ')]

    def update(self) -> py_trees.common.Status:
        # Execute States
        self.logger.debug(f"Current State {self.current_state}")
        result = py_trees.common.Status.FAILURE
        result = py_trees.common.Status.FAILURE
        if self.current_state == DoorRequestState.IDLE:
            for door in self.requested_doors:
                self.requested_doors_list += door + " "
                self.publish_door_mode(door)
            if not self.door_ids:
                self.feedback_message = "Executing all doors with mode " + self.requested_door_mode_state
            else:
                self.feedback_message = "Executing  doors:" + self.requested_doors_list + "with mode " + self.requested_door_mode_state
            self.current_state = DoorRequestState.DOOR_MODE_REQUESTED
            result = py_trees.common.Status.RUNNING
        elif self.current_state == DoorRequestState.DOOR_MODE_REQUESTED:
            for door in self.requested_doors:
                if door in self.doors_current_state and self.doors_current_state[door] == 1:
                    continue
                self.publish_door_mode(door)
            # check if all doors are present in doors_current_state dict
            doors_in_doors_current_state = all(key in self.doors_current_state for key in self.requested_doors)
            if doors_in_doors_current_state:
                # check if all doors reached at requested mode
                self.all_doors_executed = all(value == self.requested_door_mode_value for value in self.doors_current_state.values())
                if self.all_doors_executed:
                    self.feedback_message = "Successfully executed doors:" + self.requested_doors_list + "with mode " + self.requested_door_mode_state
                    self.current_state = DoorRequestState.DONE
                    result = py_trees.common.Status.SUCCESS
                else:
                    self.current_state = DoorRequestState.DOOR_MODE_REQUESTED
                    result = py_trees.common.Status.RUNNING
            else:
                self.current_state = DoorRequestState.DOOR_MODE_REQUESTED
                result = py_trees.common.Status.RUNNING
        elif self.current_state == DoorRequestState.DONE:
            self.logger.debug("Nothing to do!")
        else:
            self.logger.error(f"Invalid state {self.current_state}")

        return result

    def publish_door_mode(self, door):
        msg = DoorRequestMsg()
        msg.door_name = door
        msg.requested_mode.value = self.requested_door_mode_value
        self.publisher.publish(msg)

    def callback_get_door_states(self, msg):
        if msg.door_name not in self.retrieve_doors:
            self.retrieve_doors.append(msg.door_name)
        for door in self.requested_doors:
            if msg.door_name == door:
                if msg.current_mode.value == self.requested_door_mode_value:
                    self.doors_current_state[door] = self.requested_door_mode_value
                else:
                    self.doors_current_state[door] = msg.current_mode.value
