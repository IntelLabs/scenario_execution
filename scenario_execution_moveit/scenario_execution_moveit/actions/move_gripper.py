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

from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.logging import get_logger
from pymoveit2 import MoveIt2State, GripperInterface
import py_trees


class MoveGripper(py_trees.behaviour.Behaviour):

    def __init__(self, name: str, associated_actor, gripper: str):
        super().__init__(name)
        self.namespace = associated_actor["namespace"]
        self.joint_names = associated_actor["joint_names"]
        self.base_link_name = associated_actor["base_link_name"]
        self.end_effector_name = associated_actor["end_effector_name"]
        self.move_group_arm = associated_actor["move_group_arm"]
        self.move_group_gripper = associated_actor["move_group_gripper"]
        self.gripper_joint_names = associated_actor["gripper_joint_names"]
        self.gripper = gripper
        self.execute = False
        self.node = None
        self.logger = None
        self.gripper_interface = None
        self.current_state = None

    def setup(self, **kwargs):
        try:
            self.node: Node = kwargs['node']
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(
                self.name, self.__class__.__name__)
            raise KeyError(error_message) from e
        self.logger = get_logger(self.name)
        # Create MoveIt 2 interface
        self.gripper_interface = GripperInterface(
            node=self.node,
            gripper_joint_names=self.gripper_joint_names,
            open_gripper_joint_positions=[-0.037, +0.037],
            closed_gripper_joint_positions=[-0.015, 0.015],
            gripper_group_name=self.move_group_gripper,
            callback_group=ReentrantCallbackGroup(),
            gripper_command_action_name="gripper_action_controller/gripper_cmd"
        )

        if self.gripper not in ['open', 'close']:
            raise ValueError("Invalid gripper state speficied. Allowed values are 'open' or 'close'.")

    def update(self) -> py_trees.common.Status:
        self.current_state = self.gripper_interface.query_state()
        if not self.execute:
            if self.current_state == MoveIt2State.EXECUTING:
                self.logger.info("Another motion is in progress. Waiting for the current motion to complete...")
                result = py_trees.common.Status.RUNNING
            else:
                if self.gripper == 'open':
                    self.gripper_interface.open()
                else:
                    self.gripper_interface.close()
                self.logger.info(f"No motion in progress. Setting gripper to {self.gripper} state...")
                self.execute = True
                result = py_trees.common.Status.RUNNING
        else:
            if self.current_state == MoveIt2State.IDLE:
                self.logger.info(f"Gripper state {self.gripper} set successfully.")
                result = py_trees.common.Status.SUCCESS
                self.execute = False  # Reset for potential future executions
            elif self.current_state == MoveIt2State.EXECUTING:
                self.logger.info("Motion in progress...")
                result = py_trees.common.Status.RUNNING
            else:
                self.logger.info("Unknown state encountered while executing motion. Requesting again...")
                result = py_trees.common.Status.RUNNING
        return result
