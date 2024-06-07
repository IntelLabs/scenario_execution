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
        self.synchronous = True
        self.cancel_after_secs = 0.0
        self.gripper_interface = None
        self.current_state = None

    def setup(self, **kwargs):
        try:
            self.node: Node = kwargs['node']
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(
                self.name, self.__class__.__name__)
            raise KeyError(error_message) from e

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

    def update(self) -> py_trees.common.Status:
        self.current_state = self.gripper_interface.query_state()
        self.logger.info(f"Current State: {self.current_state}")
        if self.current_state == MoveIt2State.IDLE:
            if not self.execute:
                self.gripper_interface.open()
                result = py_trees.common.Status.RUNNING
            else:
                result = py_trees.common.Status.SUCCESS
        elif self.current_state == MoveIt2State.EXECUTING:
            self.logger.info(f"Executing gripper....")
            result = py_trees.common.Status.RUNNING
            self.execute = True
        else:
            self.logger.info(f"Requesting gripper pose....")
            result = py_trees.common.Status.RUNNING

        return result
