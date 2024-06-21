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

from .nav2_common import get_pose_stamped
from .ros_action_call import RosActionCall, ActionCallActionState
from nav2_msgs.action import NavigateToPose


class NavToPose(RosActionCall):
    """
    Class to navigate to a pose
    """

    def __init__(self, name: str, associated_actor, goal_pose: list, monitor_progress: bool, action_topic: str, namespace_override: str) -> None:
        self.namespace = associated_actor["namespace"]
        if namespace_override:
            self.namespace = namespace_override
        self.goal_pose = goal_pose
        super().__init__(name, self.namespace + '/' + action_topic, "nav2_msgs.action.NavigateToPose", "")

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
                feedback_message = 'Estimated time of arrival: ' + \
                    '{0:.0f}'.format(Duration.from_msg(self.received_feedback.estimated_time_remaining).nanoseconds / 1e9) + ' seconds.'
            else:
                goal_msg = self.get_goal_msg()
                feedback_message = f"Executing navigation to ({goal_msg.pose.pose.position.x}, {goal_msg.pose.pose.position.y})."
        elif current_state == ActionCallActionState.DONE:
            feedback_message = f"Goal reached."
        return feedback_message
