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

import time
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.logging import get_logger
from pymoveit2 import MoveIt2State
from scenario_execution_moveit.moveit_common import Gripper
from scenario_execution.actions.base_action import BaseAction
from moveit_msgs.srv import GetMotionPlan
import py_trees
import ast


class MoveGripper(BaseAction):

    def __init__(self, associated_actor, gripper: str):
        super().__init__()
        self.namespace = associated_actor["namespace"]
        self.joint_names = associated_actor["joint_names"]
        self.base_link_name = associated_actor["base_link_name"]
        self.end_effector_name = associated_actor["end_effector_name"]
        self.move_group_arm = associated_actor["move_group_arm"]
        self.move_group_gripper = associated_actor["move_group_gripper"]
        self.gripper_joint_names = associated_actor["gripper_joint_names"]
        self.open_gripper_position = ast.literal_eval(associated_actor["open_gripper_position"])
        self.close_gripper_position = ast.literal_eval(associated_actor["close_gripper_position"])
        self.gripper = gripper
        self.execute = False
        self.node = None
        self.gripper_interface = None
        self.current_state = None
        self.logger = None
        self.service_available = False
        self.state = None
        # timeout
        self.service_start_time = None
        self.execution_start_time = None
        self.timeout_duration = 30  # Timeout duration in seconds

    def setup(self, **kwargs):
        try:
            self.node: Node = kwargs['node']
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(
                self.name, self.__class__.__name__)
            raise KeyError(error_message) from e
        self.logger = get_logger(self.name)

        # Create MoveIt 2 interface
        self.gripper_interface = Gripper(
            node=self.node,
            gripper_joint_names=self.gripper_joint_names,
            open_gripper_joint_positions=self.open_gripper_position,
            closed_gripper_joint_positions=self.close_gripper_position,
            gripper_group_name=self.move_group_gripper,
            callback_group=ReentrantCallbackGroup(),
            gripper_command_action_name="gripper_action_controller/gripper_cmd"
        )

        if self.gripper not in ['open', 'close']:
            raise ValueError("Invalid gripper state speficied. Allowed values are 'open' or 'close'.")

        self.client = self.node.create_client(GetMotionPlan, 'plan_kinematic_path')

    def update(self) -> py_trees.common.Status:     # pylint: disable=R0911
        self.current_state = self.gripper_interface.query_state()
        result = py_trees.common.Status.RUNNING
        self.wait_for_service_plan_kinematic_path()
        # Handle service availability
        if not self.service_available:
            if self.service_start_time is None:
                self.service_start_time = time.time()
            if self.check_timeout(self.service_start_time):
                self.feedback_message = f"Timeout waiting for MoveIt to become active."  # pylint: disable= attribute-defined-outside-init
                return py_trees.common.Status.FAILURE
            self.feedback_message = "Waiting for MoveIt to become active..."  # pylint: disable= attribute-defined-outside-init
            return result
        # Handle executing state
        if self.current_state == MoveIt2State.EXECUTING:
            if not self.execute:
                self.feedback_message = "Another motion is in progress. Waiting for current motion to complete..."   # pylint: disable= attribute-defined-outside-init
                return result
            else:
                self.feedback_message = f"Motion gripper {self.gripper} in progress..."  # pylint: disable= attribute-defined-outside-init
                return result
        # Handle idle state
        if self.current_state == MoveIt2State.IDLE:
            if not self.execute:
                self.logger.info(f"No motion in progress. Initiating gripper to {self.gripper} state...")
                if self.gripper == 'open':
                    self.gripper_interface.open()
                else:
                    self.gripper_interface.close()
                self.feedback_message = f"Moving gripper to {self.gripper} state."  # pylint: disable= attribute-defined-outside-init
                self.execute = True
                return result
            else:
                self.logger.info(f"Gripper state {self.gripper} set successfully.")
                self.feedback_message = f"Gripper state {self.gripper} set successfully."  # pylint: disable= attribute-defined-outside-init
                return py_trees.common.Status.SUCCESS
        if self.execution_start_time is None:
            self.execution_start_time = time.time()
        if self.check_timeout(self.execution_start_time):
            self.feedback_message = f"Timeout waiting for current state."  # pylint: disable= attribute-defined-outside-init
            return py_trees.common.Status.FAILURE
        self.feedback_message = "Waiting for a valid current state..."  # pylint: disable= attribute-defined-outside-init
        return result

    def wait_for_service_plan_kinematic_path(self):
        if self.client.wait_for_service(1.0):
            self.service_available = True
        else:
            self.service_available = False

    def check_timeout(self, start_time):
        if start_time is None:
            return False
        if time.time() - start_time > self.timeout_duration:
            return True
        return False
