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

""" Scenario execution plugin for navigating through poses """

from enum import Enum

from rclpy.node import Node
from rclpy.duration import Duration

import py_trees

from nav2_simple_commander.robot_navigator import TaskResult  # pylint: disable=import-error

from .nav2_common import NamespaceAwareBasicNavigator

from geometry_msgs.msg import PoseStamped
from transforms3d.taitbryan import euler2quat


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

    Args:
        goal_pose: a 6 numbers list in str form containing the goal pose of the entity
            in the shape of [x, y, z. roll, pitch, yaw].

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

        goal_poses_as_strings = goal_poses.split(';')

        if not goal_poses_as_strings:
            raise ValueError(f"Goal poses empty.")

        self.goal_poses = []

        for goal_pose_string in goal_poses_as_strings:
            result = True
            goal_split = goal_pose_string.split(',')
            if len(goal_split) != 3:
                result = False

            if result:
                goal = [float(goal_split[0]), float(goal_split[1]), 0., 0., 0., float(goal_split[2])]
                self.goal_poses.append(goal)

            if not result:
                raise ValueError(
                    f"Goal poses '{goal_poses} invalid ({goal_pose_string}). Expected format: x1,y1,yaw1;x2,y2,yaw2;... Units: m, rad")

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
                goal_poses.append(NavThroughPoses.get_pose_stamped(self.nav.get_clock().now().to_msg(), pose))
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

    @staticmethod
    def get_pose_stamped(timestamp, pose_list: list):
        """
        Convert pose list to PoseStamped
        """
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = timestamp
        pose.pose.position.x = pose_list[0]
        pose.pose.position.y = pose_list[1]
        pose.pose.position.z = pose_list[2]

        # euler2quat() requires "zyx" convention,
        # while in YAML, we define as pitch-roll-yaw (xyz), since it's more intuitive.
        quaternion = euler2quat(pose_list[5],
                                pose_list[4],
                                pose_list[3])
        pose.pose.orientation.w = quaternion[0]
        pose.pose.orientation.x = quaternion[1]
        pose.pose.orientation.y = quaternion[2]
        pose.pose.orientation.z = quaternion[3]
        return pose
