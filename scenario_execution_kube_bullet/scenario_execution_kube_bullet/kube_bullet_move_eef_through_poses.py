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

""" Class for kube bullet send joint position action """

from enum import Enum
import py_trees
from scenario_execution_kube_bullet.kube_bullet_common import KubeBulletClient

from kube_bullet.grpc_kube_bullet import kube_bullet_grpc_pb2


class KubeBulletMoveEEFThroughPosesActionState(Enum):
    """
    States for kube bullet spawn robot action
    """
    IDLE = 1
    REQUEST_SENT = 2
    FAILURE = 3
    SUCCESS = 4


class KubeBulletMoveEEFThroughPoses(py_trees.behaviour.Behaviour):
    """
    Class for kube bullet send joint position action

    Args:
        path: a list of waypoints
    """

    def __init__(self, name, associated_actor, component: str, threshold: float, link_name: str, poses: list):
        super().__init__(name)
        self.robot_name = associated_actor["name"]
        self.active_components = associated_actor["active_components"].split(';')
        self.model = associated_actor["model"]
        self.current_state = KubeBulletMoveEEFThroughPosesActionState.IDLE
        self.component = '__'.join([self.robot_name, component])
        if not threshold:
            raise TypeError(f'threshold not initialized: got {threshold}.')
        self.threshold = threshold
        self.link_name = link_name
        poses_as_string = poses.split(';')
        if not poses_as_string:
            raise ValueError(f"No poses specified.")

        self.poses = []
        for i, pose_string in enumerate(poses_as_string):
            split_pose = pose_string.split(',')
            split_pose_length = len(split_pose)
            if split_pose_length != 6:
                raise ValueError(
                    f"Ill-formated poses list {poses} for kube_bullet: we expect poses of the form x,y,z,roll,pitch,yaw, i.e., the length of the pose list must be 6, pose {i} list has lenght: {split_pose_length}")
            pose = [float(pose_elem) for pose_elem in split_pose]
            pos = pose[:3]
            pos = kube_bullet_grpc_pb2.pos(x=pose[0], y=pose[1], z=pose[2])
            rpy = kube_bullet_grpc_pb2.rpy(r=pose[3], p=pose[4], y=pose[5])

            pose_euler = kube_bullet_grpc_pb2.pose_euler(
                position=pos,
                euler=rpy
            )

            self.poses.append(pose_euler)

        self.pose_index = -1
        self.num_poses = len(self.poses)
        self.client = None
        self.request = None
        self.current_pose = None
        self.feedback = None

        # TODO

    def setup(self, **kwargs):
        """
        Setup KubeBullet Client

        """
        self.client = KubeBulletClient()

    def update(self) -> py_trees.common.Status:  # pylint: disable=too-many-return-statements
        """
        Execute states
        """
        if self.current_state == KubeBulletMoveEEFThroughPosesActionState.IDLE:

            self.pose_index += 1
            if self.pose_index < self.num_poses:
                self.current_pose = self.poses[self.pose_index]
                self.request = kube_bullet_grpc_pb2.MoveArmThroughEefPosesRequest(
                    robot_module_name=self.component,
                    eef_link_name=self.link_name,
                    eef_poses=[self.current_pose]
                )
                self.current_state = KubeBulletMoveEEFThroughPosesActionState.REQUEST_SENT
                self.feedback_message = f"Request sent for {self.current_pose}"  # pylint: disable= attribute-defined-outside-init
                self.feedback = next(
                    self.client.stub.MoveArmThroughEefPosesWithFeedback(self.request))
            else:
                self.current_state = KubeBulletMoveEEFThroughPosesActionState.SUCCESS
                self.feedback_message = f"Reached final pose."  # pylint: disable= attribute-defined-outside-init

            return py_trees.common.Status.RUNNING

        elif self.current_state == KubeBulletMoveEEFThroughPosesActionState.REQUEST_SENT:
            self.feedback = next(self.client.stub.MoveArmThroughEefPosesWithFeedback(self.request))
            self.current_state = KubeBulletMoveEEFThroughPosesActionState.REQUEST_SENT
            remaining_dist = self.feedback.remaining_avg_joint_angle
            self.feedback_message = f"Request sent: remaining avg joint angle distance: {remaining_dist}"  # pylint: disable= attribute-defined-outside-init
            if remaining_dist < self.threshold:
                current_pose_index = self.pose_index + 1
                self.feedback_message = f"Reached current pose {current_pose_index}: remaining distance is lower than threshold, i.e.,  {remaining_dist} < {self.threshold}, go to the next pose of {self.num_poses}"  # pylint: disable= attribute-defined-outside-init
                self.current_state = KubeBulletMoveEEFThroughPosesActionState.IDLE
            return py_trees.common.Status.RUNNING
        elif self.current_state == KubeBulletMoveEEFThroughPosesActionState.SUCCESS:
            self.feedback_message = f"Reached success"  # pylint: disable= attribute-defined-outside-init
            return py_trees.common.Status.SUCCESS

        self.feedback_message = f"Failed"  # pylint: disable= attribute-defined-outside-init
        return py_trees.common.Status.FAILURE
