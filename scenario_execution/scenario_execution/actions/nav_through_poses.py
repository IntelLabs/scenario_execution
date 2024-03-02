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

import py_trees

from nav2_simple_commander.robot_navigator import TaskResult  # pylint: disable=import-error

from .nav2_common import NamespaceAwareBasicNavigator
from .nav2_common import get_pose_stamped


class NavThroughPosesState(Enum):
    """
    States for executing a nav-through-poses with nav2
    """
    IDLE = 1
    NAV_TO_GOAL = 2
    DONE = 3


class NavThroughPoses(py_trees.behaviour.Behaviour):
    """
    Class to navigate through poses
    """

    def __init__(self, name: str, associated_actor, goal_poses: list, monitor_progress: bool, namespace_override: str):
        super().__init__(name)
        self.namespace = associated_actor["namespace"]
        self.monitor_progress = monitor_progress
        self.node = None
        self.future = None
        self.current_state = NavThroughPosesState.IDLE
        self.nav = None
        if namespace_override:
            self.namespace = namespace_override

        if not isinstance(goal_poses, list):
            raise TypeError(f'goal_poses needs to be list of position_3d, got {type(goal_poses)}.')
        else:
            self.goal_poses = goal_poses

    def setup(self, **kwargs):
        """
        Setup ROS2 node and service client

        """
        try:
            self.node: Node = kwargs['node']
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(
                self.name, self.__class__.__name__)
            raise KeyError(error_message) from e  # 'direct cause' traceability

        self.nav = NamespaceAwareBasicNavigator(
            node_name="basic_nav_nav_through_poses", namespace=self.namespace)

    def update(self) -> py_trees.common.Status:
        """
        Execute states
        """
        self.logger.debug(f"Current State {self.current_state}")
        result = py_trees.common.Status.FAILURE
        if self.current_state == NavThroughPosesState.IDLE:
            self.current_state = NavThroughPosesState.NAV_TO_GOAL
            goal_poses = []
            for pose in self.goal_poses:
                goal_poses.append(get_pose_stamped(self.nav.get_clock().now().to_msg(), pose))
            self.feedback_message = "Execute navigation."  # pylint: disable= attribute-defined-outside-init
            go_to_pose_result = self.nav.goThroughPoses(goal_poses)
            if go_to_pose_result:
                if self.monitor_progress:
                    result = py_trees.common.Status.RUNNING
                    self.current_state = NavThroughPosesState.NAV_TO_GOAL
                else:
                    result = py_trees.common.Status.SUCCESS
                    self.current_state = NavThroughPosesState.DONE
            else:
                self.current_state = NavThroughPosesState.DONE
                result = py_trees.common.Status.FAILURE
        elif self.current_state == NavThroughPosesState.NAV_TO_GOAL:
            if not self.nav.isTaskComplete():
                feedback = self.nav.getFeedback()
                if feedback:
                    self.feedback_message = 'Estimated time of arrival: ' + '{0:.0f}'.format(  # pylint: disable= attribute-defined-outside-init
                        Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9) + ' seconds.'

                    if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600):
                        self.nav.cancelTask()
                result = py_trees.common.Status.RUNNING
            else:
                self.current_state = NavThroughPosesState.DONE
                result = self.nav.getResult()
                if result == TaskResult.SUCCEEDED:
                    self.feedback_message = 'Goal succeeded!'  # pylint: disable= attribute-defined-outside-init
                    result = py_trees.common.Status.SUCCESS
                elif result == TaskResult.CANCELED:
                    self.feedback_message = 'Goal was canceled!'  # pylint: disable= attribute-defined-outside-init
                    result = py_trees.common.Status.FAILURE
                elif result == TaskResult.FAILED:
                    self.feedback_message = 'Goal failed!'  # pylint: disable= attribute-defined-outside-init
                    result = py_trees.common.Status.FAILURE
        elif self.current_state == NavThroughPosesState.DONE:
            self.logger.debug("Nothing to do!")
        else:
            self.logger.error(f"Invalid state {self.current_state}")

        return result
