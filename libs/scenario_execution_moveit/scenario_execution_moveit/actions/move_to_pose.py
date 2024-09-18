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
from moveit_msgs.msg import MotionPlanRequest, PlanningOptions, Constraints, PositionConstraint, OrientationConstraint, BoundingVolume
from geometry_msgs.msg import PoseStamped
from shape_msgs.msg import SolidPrimitive


class MoveToPose(RosActionCall):
    """
    Class to move to a pose
    """

    def __init__(self, associated_actor, action_topic: str, namespace_override: str, success_on_acceptance: bool) -> None:
        self.namespace = associated_actor["namespace"]
        if namespace_override:
            self.namespace = namespace_override
        self.goal_pose = None
        super().__init__(self.namespace + '/' + action_topic, "moveit_msgs.action.MoveGroup", success_on_acceptance=success_on_acceptance)

    def execute(self, associated_actor, goal_pose: list, plan_only: bool, tolerance: float, replan: bool, max_velocity_scaling_factor: float) -> None:  # pylint: disable=arguments-differ,arguments-renamed
        self.goal_pose = goal_pose
        self.group = associated_actor['arm_group']
        self.base_link = associated_actor['base_link']
        self.end_effector = associated_actor['end_effector']
        self.plan_only = plan_only
        self.tolerance = tolerance
        self.replan = replan
        self.max_velocity_scaling_factor = max_velocity_scaling_factor
        super().execute("")

    def get_goal_msg(self):
        motion_plan_request = MotionPlanRequest()
        motion_plan_request.group_name = self.group
        motion_plan_request.max_velocity_scaling_factor = self.max_velocity_scaling_factor

        target_pose = PoseStamped()
        target_pose.header.frame_id = self.base_link  # reference_frame
        target_pose.pose.position.x = float(self.goal_pose[0])
        target_pose.pose.position.y = float(self.goal_pose[1])
        target_pose.pose.position.z = float(self.goal_pose[2])
        target_pose.pose.orientation.x = float(self.goal_pose[3])
        target_pose.pose.orientation.y = float(self.goal_pose[4])
        target_pose.pose.orientation.z = float(self.goal_pose[5])
        target_pose.pose.orientation.w = float(self.goal_pose[6])

        # Create PositionConstraint
        position_constraint = PositionConstraint()
        position_constraint.header.frame_id = self.base_link  # Reference frame
        position_constraint.link_name = self.end_effector  # Link for which this constraint applies ( End-Effector Name)

        # Define the region around the target position (e.g., a small bounding box or sphere around the target)
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.SPHERE
        bounding_volume = BoundingVolume()
        bounding_volume.primitives.append(primitive)
        bounding_volume.primitive_poses.append(target_pose.pose)

        # Assign the bounding volume to the position constraint
        position_constraint.constraint_region = bounding_volume

        # Optionally, set tolerances for weight and target position constraint
        position_constraint.weight = 1.0  # Importance of this constraint

        # Create OrientationConstraint
        orientation_constraint = OrientationConstraint()
        orientation_constraint.header.frame_id = self.base_link  # Reference frame
        orientation_constraint.link_name = self.end_effector  # Link for which this constraint applies ( End-Effector Name)
        orientation_constraint.orientation = target_pose.pose.orientation
        orientation_constraint.absolute_x_axis_tolerance = self.tolerance  # Tolerances for orientation
        orientation_constraint.absolute_y_axis_tolerance = self.tolerance
        orientation_constraint.absolute_z_axis_tolerance = self.tolerance
        orientation_constraint.weight = 1.0  # Importance of this constraint

        # Create the Constraints object and add both position and orientation constraints
        goal_constraints = Constraints()
        goal_constraints.position_constraints.append(position_constraint)
        goal_constraints.orientation_constraints.append(orientation_constraint)
        motion_plan_request.goal_constraints.append(goal_constraints)

        planning_options = PlanningOptions()
        planning_options.plan_only = self.plan_only  # Only plan, do not execute (if False, action will execute also)
        planning_options.replan = self.replan  # if True, re-planning in case of failure
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
