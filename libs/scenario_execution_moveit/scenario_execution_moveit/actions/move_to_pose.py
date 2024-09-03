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
from rclpy.action.client import GoalStatus
from pymoveit2 import MoveIt2State
from scenario_execution_moveit.moveit_common import MoveIt2Interface
from scenario_execution.actions.base_action import BaseAction
from moveit_msgs.srv import GetMotionPlan
from moveit_msgs.msg import MoveItErrorCodes
import py_trees


class MoveToPose(BaseAction):

    def __init__(self, associated_actor, goal_pose: list):
        super().__init__()
        self.namespace = associated_actor["namespace"]
        self.joint_names = associated_actor["joint_names"]
        self.base_link_name = associated_actor["base_link_name"]
        self.end_effector_name = associated_actor["end_effector_name"]
        self.move_group_arm = associated_actor["move_group_arm"]
        self.position = [goal_pose['position']['x'], goal_pose['position']['y'], goal_pose['position']['z']]
        self.orientation = [goal_pose['orientation']['roll'], goal_pose['orientation']
                            ['pitch'], goal_pose['orientation']['yaw'], 1.000000000]
        self.cartesian = False
        self.cartesian_max_step = 0.025
        self.cartesian_fraction_threshold = 0.0
        self.cartesian_jump_threshold = 0.0
        self.cartesian_avoid_collisions = False
        self.execute = False
        self.node = None
        self.moveit2 = None
        self.current_state = None
        self.logger = None
        self.service_available = False
        self.goal_status = None
        self.last_error_code = None
        # timeout
        self.service_start_time = None
        self.execution_start_time = None
        self.retry_start_time = None
        self.timeout_duration = 30  # Timeout duration in seconds
        self.future_called = False

    def setup(self, **kwargs):
        try:
            self.node: Node = kwargs['node']
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(
                self.name, self.__class__.__name__)
            raise KeyError(error_message) from e

        self.logger = get_logger(self.name)

        # Create MoveIt 2 interface
        self.moveit2 = MoveIt2Interface(
            node=self.node,
            joint_names=self.joint_names,
            base_link_name=self.namespace + '/' + self.base_link_name,
            end_effector_name=self.namespace + '/' + self.end_effector_name,
            group_name=self.move_group_arm,
            callback_group=ReentrantCallbackGroup()
        )

        self.moveit2.planner_id = "RRTConnect"

        # Scale down velocity and acceleration of joints (percentage of maximum)
        self.moveit2.max_velocity = 0.5
        self.moveit2.max_acceleration = 0.5

        self.moveit2.cartesian_avoid_collisions = self.cartesian_avoid_collisions
        self.moveit2.cartesian_jump_threshold = self.cartesian_jump_threshold

        self.client = self.node.create_client(GetMotionPlan, 'plan_kinematic_path')

    def update(self) -> py_trees.common.Status:     # pylint: disable=R0911
        self.current_state = self.moveit2.query_state()
        self.last_error_code = self.moveit2.get_last_execution_error_code()
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
                self.feedback_message = "Motion to pose in progress..."  # pylint: disable= attribute-defined-outside-init
                return result
        # Handle idle state
        if self.current_state == MoveIt2State.IDLE:
            if not self.execute:
                self.logger.info("No motion in progress. Initiating move to pose...")
                self.feedback_message = f"Moving to pose."  # pylint: disable= attribute-defined-outside-init
                self.move_to_pose()
                self.execute = True
                return result
            else:
                if not self.future_called:
                    future = self.moveit2.get_execution_future()
                    future.add_done_callback(self.future_done_callback)
                    self.future_called = True
                if self.goal_status is not None:
                    if self.goal_status == GoalStatus.STATUS_SUCCEEDED:
                        self.logger.info("Motion to pose successful.")
                        self.feedback_message = "Motion to pose successful."  # pylint: disable= attribute-defined-outside-init
                        return py_trees.common.Status.SUCCESS
                    elif self.goal_status == GoalStatus.STATUS_CANCELED:
                        self.feedback_message = f"Goal canceled."   # pylint: disable= attribute-defined-outside-init
                        return py_trees.common.Status.FAILURE
                    elif self.goal_status == GoalStatus.STATUS_ABORTED:
                        self.feedback_message = f"Goal aborted."   # pylint: disable= attribute-defined-outside-init
                        return py_trees.common.Status.FAILURE
                    else:
                        self.feedback_message = f"Goal status unknown."   # pylint: disable= attribute-defined-outside-init
                        return py_trees.common.Status.FAILURE
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

    def future_done_callback(self, future):
        if not future.result():
            return
        self.goal_status = future.result().status

    def check_timeout(self, start_time):
        if start_time is None:
            return False
        if time.time() - start_time > self.timeout_duration:
            return True
        return False

    def log_moveit_error(self, error_code):
        error_code_mapping = {v: k for k, v in MoveItErrorCodes.__dict__.items() if isinstance(v, int)}
        description = error_code_mapping.get(error_code.val, "UNKNOWN_ERROR_CODE")
        return description

    def move_to_pose(self):
        self.moveit2.move_to_pose(
            position=self.position,
            quat_xyzw=self.orientation,
            cartesian=self.cartesian,
            cartesian_max_step=self.cartesian_max_step,
            cartesian_fraction_threshold=self.cartesian_fraction_threshold,
        )
