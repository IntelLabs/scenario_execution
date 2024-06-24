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

from time import sleep
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.logging import get_logger
from pymoveit2 import MoveIt2, MoveIt2State
import py_trees
import ast


class MoveToJointPose(py_trees.behaviour.Behaviour):

    def __init__(self, name: str, associated_actor, joint_pose: str):
        super().__init__(name)
        self.namespace = associated_actor["namespace"]
        self.joint_names = associated_actor["joint_names"]
        self.base_link_name = associated_actor["base_link_name"]
        self.end_effector_name = associated_actor["end_effector_name"]
        self.move_group_arm = associated_actor["move_group_arm"]
        self.joint_pose = ast.literal_eval(joint_pose)
        self.execute = False
        self.node = None
        self.moveit2 = None
        self.current_state = None
        self.logger = None

    def setup(self, **kwargs):
        try:
            self.node: Node = kwargs['node']
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(
                self.name, self.__class__.__name__)
            raise KeyError(error_message) from e
        self.logger = get_logger(self.name)
        # Create MoveIt 2 interface
        self.moveit2 = MoveIt2(
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

    def update(self) -> py_trees.common.Status:
        self.current_state = self.moveit2.query_state()
        result = py_trees.common.Status.RUNNING
        if not self.execute:
            if self.current_state == MoveIt2State.EXECUTING:
                self.logger.info("Another motion is in progress. Waiting for current motion to complete...")
                result = py_trees.common.Status.RUNNING
            else:
                self.logger.info("No motion in progress. Initiating move to joint pose...")
                self.feedback_message = f"Moving to joint pose {self.joint_pose}."  # pylint: disable= attribute-defined-outside-init
                self.moveit2.move_to_configuration(self.joint_pose)
                result = py_trees.common.Status.RUNNING
                self.execute = True
        elif self.current_state == MoveIt2State.EXECUTING:
            future = self.moveit2.get_execution_future()
            if future:
                while not future.done():
                    self.logger.info("Motion to joint pose in progress...")
                    sleep(1)
                if str(future.result().status) == '4':
                    self.logger.info("Motion to joint pose successful.")
                    result = py_trees.common.Status.SUCCESS
                else:
                    self.logger.info(f"{str(future.result().result.error_code)}")
                    result = py_trees.common.Status.FAILURE
            else:
                self.logger.info("Waiting for response from arm...")
                result = py_trees.common.Status.RUNNING
        elif self.current_state == MoveIt2State.IDLE:
            self.logger.info("Postion not reachable or arm is already at the specified position!")
            result = py_trees.common.Status.FAILURE

        return result
