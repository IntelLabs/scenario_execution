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

""" Class for kube bullet spawn object action """

from enum import Enum
import py_trees
from scenario_execution_kube_bullet.kube_bullet_common import KubeBulletClient

from kube_bullet.grpc_kube_bullet import kube_bullet_grpc_pb2


class KubeBulletSpawnObjectActionState(Enum):
    """
    States for kube bullet spawn object action
    """
    IDLE = 1
    REQUEST_SENT = 2
    FAILURE = 3


class KubeBulletSpawnObject(py_trees.behaviour.Behaviour):
    """
    Class for kube bullet spawn object action

    Args:
        path: a list of waypoints
    """

    def __init__(self, name, associated_actor, position: list, flags: list):
        super().__init__(name)
        self.object_name = associated_actor["name"]
        self.model = associated_actor["model"]

        position_as_string = position.split(';')
        if not position_as_string:
            raise ValueError(f"No position specified.")

        position_length = len(position_as_string)
        if position_length != 3:
            raise ValueError(
                f"Ill-formated position {position} for kube_bullet object: we expect positions of lentgh 3, given position has length: {position_length}")
        self.position = [float(position_string) for position_string in position_as_string]

        self.flags = flags.split(';')
        self.current_state = KubeBulletSpawnObjectActionState.IDLE
        self.client = None

    def setup(self, **kwargs):
        """
        Setup KubeBulletClient

        """
        self.client = KubeBulletClient()

    def update(self) -> py_trees.common.Status:  # pylint: disable=too-many-return-statements
        """
        Execute states
        """
        if self.current_state == KubeBulletSpawnObjectActionState.IDLE:

            self.client.stub.SetupObject(kube_bullet_grpc_pb2.ObjectSetupRequest(
                command='spawn',
                object_name=self.object_name,
                config_path=self.model,
                position=self.position,
                flags=self.flags
            ))

            self.current_state = KubeBulletSpawnObjectActionState.REQUEST_SENT
            self.feedback_message = f"Request sent."  # pylint: disable= attribute-defined-outside-init
            return py_trees.common.Status.RUNNING
        elif self.current_state == KubeBulletSpawnObjectActionState.REQUEST_SENT:
            self.feedback_message = f"Success"  # pylint: disable= attribute-defined-outside-init
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE
