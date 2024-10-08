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

from rclpy.duration import Duration
from scenario_execution_ros.actions.ros_action_call import RosActionCall, ActionCallActionState
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, Constraints, JointConstraint, PlanningOptions


class MoveToJointPose(RosActionCall):
    """
    Class to move to a joint pose
    """

    def __init__(self, associated_actor, action_topic: str, namespace_override: str, success_on_acceptance: bool) -> None:
        self.namespace = associated_actor["namespace"]
        if namespace_override:
            self.namespace = namespace_override
        self.goal_pose = None
        self.move_group = None
        self.group = None
        self.join_names = None
        super().__init__(self.namespace + '/' + action_topic, "moveit_msgs.action.MoveGroup", success_on_acceptance=success_on_acceptance)

    def execute(self, associated_actor, goal_pose: list, move_group: str, plan_only: bool, tolerance: float, replan: bool, max_velocity_scaling_factor: float) -> None:  # pylint: disable=arguments-differ,arguments-renamed
        self.goal_pose = goal_pose

        if not isinstance(move_group, tuple) or not isinstance(move_group[0], str):
            raise ValueError("move group type expected to be enum.")
        self.move_group = move_group[0]
        if self.move_group == "arm":
            self.group = associated_actor['arm_group']
            self.join_names = associated_actor['arm_joints']
        elif self.move_group == "gripper":
            self.group = associated_actor['gripper_group']
            self.join_names = associated_actor['gripper_joints']
        else:
            raise ValueError(f"element_type {move_group[0]} unknown.")
        self.plan_only = plan_only
        self.tolerance = tolerance
        self.replan = replan
        self.max_velocity_scaling_factor = max_velocity_scaling_factor
        super().execute("")

    def get_goal_msg(self):
        motion_plan_request = MotionPlanRequest()
        motion_plan_request.group_name = self.group
        motion_plan_request.max_velocity_scaling_factor = self.max_velocity_scaling_factor
        constraints = Constraints()
        for joint_name, position in zip(self.join_names, self.goal_pose):
            joint_constraint = JointConstraint()
            joint_constraint.joint_name = joint_name
            joint_constraint.position = float(position)
            joint_constraint.tolerance_above = self.tolerance
            joint_constraint.tolerance_below = self.tolerance
            joint_constraint.weight = 1.0  # Set weight (importance) of this constraint
            constraints.joint_constraints.append(joint_constraint)

        motion_plan_request.goal_constraints.append(constraints)
        planning_options = PlanningOptions()
        planning_options.plan_only = self.plan_only  # Only plan, do not execute (set to False if you want to execute)
        planning_options.replan = self.replan  # Set to True if you want to allow re-planning in case of failure
        goal_msg = MoveGroup.Goal()
        goal_msg.request = motion_plan_request  # Add the motion planning request
        goal_msg.planning_options = planning_options  # Add the planning options
        return goal_msg

    def get_feedback_message(self, current_state):
        feedback_message = super().get_feedback_message(current_state)

        if self.current_state == ActionCallActionState.IDLE:
            feedback_message = "Waiting for manipulation"
        elif self.current_state == ActionCallActionState.ACTION_CALLED:
            if self.received_feedback:
                feedback_message = f'Estimated time of arrival: {Duration.from_msg(self.received_feedback.estimated_time_remaining).nanoseconds / 1e9:0.0f}.'
            else:
                feedback_message = f"Executing manipulation to ({self.goal_pose})."
        elif current_state == ActionCallActionState.DONE:
            feedback_message = f"Goal reached."
        return feedback_message
