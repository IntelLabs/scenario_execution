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

from pymoveit2 import MoveIt2, MoveIt2State
from arm_sim_scenario.robots import wx200
import py_trees
import ast


class MoveToPose(py_trees.behaviour.Behaviour):

    def __init__(self, name: str, associated_actor, goal_pose: list):
        super().__init__(name)
        self.namespace = associated_actor["namespace"]
        self.joint_names = associated_actor["joint_names"]
        self.base_link_name = associated_actor["base_link_name"]
        self.end_effector_name = associated_actor["end_effector_name"]
        self.move_group_arm = associated_actor["move_group_arm"]
        self.position = [goal_pose['position']['x'], goal_pose['position']['y'], goal_pose['position']['z']]
        self.orientation = [goal_pose['orientation']['roll'], goal_pose['orientation']['pitch'], goal_pose['orientation']['yaw'], 1.000000000]
        self.cartesian = False
        self.cartesian_max_step = 0.025
        self.cartesian_fraction_threshold = 0.0
        self.cartesian_jump_threshold = 0.0
        self.cartesian_avoid_collisions = False
        self.execute = False
        

    def setup(self, **kwargs):
        try:
            self.node: Node = kwargs['node']
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(
                self.name, self.__class__.__name__)
            raise KeyError(error_message) from e

        self.synchronous = True

        # # If non-positive, don't cancel. Only used if synchronous is False
        self.cancel_after_secs = 0.0

        # Create MoveIt 2 interface
        self.moveit2 = MoveIt2(
            node= self.node,
            joint_names= self.joint_names,
            base_link_name= self.namespace + '/' + self.base_link_name,
            end_effector_name= self.namespace + '/' + self.end_effector_name,
            group_name= self.move_group_arm,
            callback_group=ReentrantCallbackGroup()
        )

        self.moveit2.planner_id = "RRTConnectkConfigDefault"

        # Scale down velocity and acceleration of joints (percentage of maximum)
        self.moveit2.max_velocity = 0.5
        self.moveit2.max_acceleration = 0.5
        self.moveit2.cartesian_avoid_collisions = self.cartesian_avoid_collisions
        self.moveit2.cartesian_jump_threshold = self.cartesian_jump_threshold

    def update(self) -> py_trees.common.Status:
        self.current_state = self.moveit2.query_state()
        if (self.current_state == MoveIt2State.IDLE):
            if (self.execute == False):
                self.move_to_pose()
                result = py_trees.common.Status.RUNNING
            else:
                result = py_trees.common.Status.SUCCESS
        elif (self.current_state == MoveIt2State.EXECUTING):
            self.logger.info(f"Executing joint pose....")
            result = py_trees.common.Status.RUNNING
            self.execute = True
        else:
            self.logger.info(f"Requesting joint pose....")
            result = py_trees.common.Status.RUNNING

        return result
    

    def move_to_pose(self):
        self.moveit2.move_to_pose(
            position=self.position,
            quat_xyzw=self.orientation,
            cartesian=self.cartesian,
            cartesian_max_step=self.cartesian_max_step,
            cartesian_fraction_threshold=self.cartesian_fraction_threshold,
        )

