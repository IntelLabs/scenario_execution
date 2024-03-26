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

from enum import Enum

from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.action import ActionClient

import py_trees

from .nav2_common import get_pose_stamped
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus


class NavToPoseState(Enum):
    """
    States for executing a nav-to-pose with nav2
    """
    IDLE = 1
    NAV_TO_GOAL_REQUESTED = 2
    NAV_TO_GOAL = 3
    DONE = 4
    FAILURE = 5


class NavToPose(py_trees.behaviour.Behaviour):
    """
    Class to navigate to a pose
    """

    def __init__(self, name: str, associated_actor, goal_pose: list, monitor_progress: bool, action_topic: str, namespace_override: str) -> None:
        super().__init__(name)
        self.goal_pose = goal_pose
        self.namespace = associated_actor["namespace"]
        if namespace_override:
            self.namespace = namespace_override

        self.monitor_progress = monitor_progress
        self.action_topic = action_topic
        self.node = None
        self.future = None
        self.current_state = NavToPoseState.IDLE
        self.nav_to_pose_client = None
        self.result_future = None
        self.goal_handle = None
        self.feedback = None
        self.request_result = None
        self.status = None

    def setup(self, **kwargs):
        """
        Setup ROS2 node and service client

        """
        try:
            self.node: Node = kwargs['node']
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(
                self.name, self.__class__.__name__)
            raise KeyError(error_message) from e

        self.nav_to_pose_client = ActionClient(
            self.node, NavigateToPose, self.namespace + '/' + self.action_topic)

        self.logger.debug("Waiting for 'NavigateToPose' action server")
        wait_for_action_server_time = 30
        try:
            while wait_for_action_server_time > 0 and not self.nav_to_pose_client.wait_for_server(timeout_sec=1.0):
                self.logger.info(f"'NavigateToPose' action server not available, waiting {wait_for_action_server_time}s...")
                wait_for_action_server_time -= 1
        except KeyboardInterrupt:
            pass
        if wait_for_action_server_time == 0:
            raise ValueError("Timeout while waiting for action server.")

    def update(self) -> py_trees.common.Status:
        """
        Execute states
        """
        self.logger.debug(f"Current State {self.current_state}")
        result = py_trees.common.Status.FAILURE
        if self.current_state == NavToPoseState.IDLE:
            goal_pose = get_pose_stamped(self.node.get_clock().now().to_msg(), self.goal_pose)
            self.feedback_message = "Executing navigation."  # pylint: disable= attribute-defined-outside-init

            result = py_trees.common.Status.RUNNING
            self.current_state = NavToPoseState.NAV_TO_GOAL_REQUESTED
            self.navigate_to_pose(goal_pose)
        elif self.current_state == NavToPoseState.NAV_TO_GOAL_REQUESTED:
            if self.goal_handle:
                if not self.goal_handle.accepted:
                    self.current_state = NavToPoseState.FAILURE
                    self.feedback_message = 'Goal was rejected!'  # pylint: disable= attribute-defined-outside-init
                else:
                    if self.monitor_progress:
                        result = py_trees.common.Status.RUNNING
                        self.current_state = NavToPoseState.NAV_TO_GOAL

                        self.result_future = self.goal_handle.get_result_async()
                    else:
                        result = py_trees.common.Status.SUCCESS
                        self.current_state = NavToPoseState.DONE
            else:
                result = py_trees.common.Status.RUNNING
        elif self.current_state == NavToPoseState.NAV_TO_GOAL:
            if not self.is_task_complete():
                if self.feedback:
                    self.feedback_message = 'Estimated time of arrival: ' + '{0:.0f}'.format(  # pylint: disable= attribute-defined-outside-init
                        Duration.from_msg(self.feedback.estimated_time_remaining).nanoseconds / 1e9) + ' seconds.'

                    if Duration.from_msg(self.feedback.navigation_time) > Duration(seconds=600):
                        self.cancel_task()
                result = py_trees.common.Status.RUNNING
            else:
                self.current_state = NavToPoseState.DONE
                result = self.status
                if result == GoalStatus.STATUS_SUCCEEDED:
                    self.feedback_message = 'Goal succeeded!'  # pylint: disable= attribute-defined-outside-init
                    result = py_trees.common.Status.SUCCESS
                elif result == GoalStatus.STATUS_CANCELED:
                    self.feedback_message = 'Goal was canceled!'  # pylint: disable= attribute-defined-outside-init
                    result = py_trees.common.Status.FAILURE
                elif result == GoalStatus.STATUS_ABORTED:
                    self.feedback_message = 'Goal failed!'  # pylint: disable= attribute-defined-outside-init
                    result = py_trees.common.Status.FAILURE
        elif self.current_state == NavToPoseState.DONE:
            self.logger.debug("Nothing to do!")
        else:
            self.logger.error(f"Invalid state {self.current_state}")

        return result

    def navigate_to_pose(self, pose, behavior_tree=''):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        goal_msg.behavior_tree = behavior_tree

        self.logger.info(
            f'Navigating to goal: {pose.pose.position.x:.2f} {pose.pose.position.y:.2f}...')
        future = self.nav_to_pose_client.send_goal_async(goal_msg, self.feedback_callback)
        future.add_done_callback(self.send_goal_done_callback)

    def send_goal_done_callback(self, future):
        self.goal_handle = future.result()

    def is_task_complete(self):
        if not self.result_future:
            return True
        if self.result_future.result():
            self.status = self.result_future.result().status
            if self.status != GoalStatus.STATUS_SUCCEEDED:
                self.feedback_message = f'Task failed with status code: {self.status}'  # pylint: disable= attribute-defined-outside-init
                return True
        else:
            # Timed out, still processing, not complete yet
            return False

        self.feedback_message = "Navigation executed."  # pylint: disable= attribute-defined-outside-init
        return True

    def cancel_task(self):
        if self.result_future:
            self.logger.info('Canceling current task.')
            self.goal_handle.cancel_goal()

    def feedback_callback(self, msg):
        self.feedback = msg.feedback

    def shutdown(self):
        self.cancel_task()
        self.nav_to_pose_client.destroy()
