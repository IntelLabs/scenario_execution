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

from scenario_execution_ros.actions.common import get_pose_stamped
from scenario_execution_ros.actions.ros_action_call import RosActionCall, ActionCallActionState
from nav2_msgs.action import NavigateToPose


class NavToPose(RosActionCall):
    """
    Class to navigate to a pose
    """

    def __init__(self, associated_actor, goal_pose: list, action_topic: str, namespace_override: str) -> None:
        self.namespace = associated_actor["namespace"]
        if namespace_override:
            self.namespace = namespace_override
        self.goal_pose = None
        super().__init__(self.namespace + '/' + action_topic, "nav2_msgs.action.NavigateToPose", "")

    def execute(self, associated_actor, goal_pose: list, action_topic: str, namespace_override: str) -> None:  # pylint: disable=arguments-differ,arguments-renamed
        self.namespace = associated_actor["namespace"]
        if namespace_override:
            self.namespace = namespace_override
        self.goal_pose = goal_pose
        super().execute(self.namespace + '/' + action_topic, "nav2_msgs.action.NavigateToPose", "")

    def get_goal_msg(self):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = get_pose_stamped(self.node.get_clock().now().to_msg(), self.goal_pose)
        return goal_msg

    def get_feedback_message(self, current_state):
        feedback_message = super().get_feedback_message(current_state)

        if self.current_state == ActionCallActionState.IDLE:
            feedback_message = "Waiting for navigation"
        elif self.current_state == ActionCallActionState.ACTION_CALLED:
            if self.received_feedback:
                feedback_message = f'Estimated time of arrival: {Duration.from_msg(self.received_feedback.estimated_time_remaining).nanoseconds / 1e9:0.0f}.'
            else:
                feedback_message = f"Executing navigation to ({self.goal_pose})."
        elif current_state == ActionCallActionState.DONE:
            feedback_message = f"Goal reached."
        return feedback_message
