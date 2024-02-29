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


class KubeBulletSendJointPositionActionState(Enum):
    """
    States for kube bullet spawn robot action
    """
    IDLE = 1
    REQUEST_SENT = 2
    FAILURE = 3


class KubeBulletSendJointPosition(py_trees.behaviour.Behaviour):
    """
    Class for kube bullet send joint position action

    Args:
        path: a list of waypoints
    """

    def __init__(self, name, associated_actor, component: str, position: str):
        super().__init__(name)
        self.robot_name = associated_actor["name"]
        self.active_components = associated_actor["active_components"].split(';')
        self.model = associated_actor["model"]
        self.current_state = KubeBulletSendJointPositionActionState.IDLE
        self.component = '__'.join([self.robot_name, component])

        position_as_string = position.split(';')
        if not position_as_string:
            raise ValueError(f"No position specified.")
        self.position = [float(position_string) for position_string in position_as_string]

        self.client = None

    def setup(self, **kwargs):
        """
        Setup KubeBullet Client

        """
        self.client = KubeBulletClient()

    def update(self) -> py_trees.common.Status:  # pylint: disable=too-many-return-statements
        """
        Execute states
        """
        if self.current_state == KubeBulletSendJointPositionActionState.IDLE:

            robot_joint_position_command = kube_bullet_grpc_pb2.RobotJointPositionCommand(
                robot_module_name=self.component,
                positions=self.position
            )
            print(robot_joint_position_command)
            res = self.client.stub.SendRobotJointPositionCommand(robot_joint_position_command)
            print(res)

            self.current_state = KubeBulletSendJointPositionActionState.REQUEST_SENT
            self.feedback_message = f"Request sent."  # pylint: disable= attribute-defined-outside-init
            return py_trees.common.Status.RUNNING
        elif self.current_state == KubeBulletSendJointPositionActionState.REQUEST_SENT:
            self.feedback_message = f"Success"  # pylint: disable= attribute-defined-outside-init
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE
