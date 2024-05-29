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

from py_moveit2 import MoveIt2, MoveIt2State
from arm_sim_scenario.robots import wx200

import py_trees
import ast


class MoveToJointPose(py_trees.behaviour.Behaviour):

    def __init__(self, name: str, joint_pose: str):
        super().__init__(name)
        self.joint_pose = ast.literal_eval(joint_pose)
        self.execute = False

    def setup(self, **kwargs):
        try:
            self.node: Node = kwargs['node']
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(
                self.name, self.__class__.__name__)
            raise KeyError(error_message) from e

        self.synchronous = True
        # If non-positive, don't cancel. Only used if synchronous is False
        self.cancel_after_secs = 0.0

        # Create MoveIt 2 interface
        self.moveit2 = MoveIt2(
            node=self.node,
            joint_names=wx200.joint_names(),
            base_link_name=wx200.base_link_name(),
            end_effector_name=wx200.end_effector_name(),
            group_name=wx200.MOVE_GROUP_ARM,
            callback_group=ReentrantCallbackGroup()
        )

        self.moveit2.planner_id = "RRTConnectkConfigDefault"

        # Scale down velocity and acceleration of joints (percentage of maximum)
        self.moveit2.max_velocity = 0.5
        self.moveit2.max_acceleration = 0.5

    def update(self) -> py_trees.common.Status:
        self.current_state = self.moveit2.query_state()
        self.logger.info(f"Current State: {self.current_state}")
        if (self.current_state == MoveIt2State.IDLE):
            if (self.execute == False):
                self.moveit2.move_to_configuration(self.joint_pose)
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

