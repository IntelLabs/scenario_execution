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

""" Class for kube bullet spawn camera action """

from enum import Enum
import py_trees
from scenario_execution_kube_bullet.kube_bullet_common import KubeBulletClient

from kube_bullet.grpc_kube_bullet import kube_bullet_grpc_pb2


class KubeBulletSpawnCameraActionState(Enum):
    """
    States for kube bullet spawn camera action
    """
    IDLE = 1
    REQUEST_SENT = 2
    FAILURE = 3


class KubeBulletSpawnCamera(py_trees.behaviour.Behaviour):
    """
    Class for kube bullet spawn camera action

    Args:
        path: a list of waypoints
    """

    def __init__(self, name, associated_actor):
        super().__init__(name)
        self.camera_name = associated_actor["name"]
        self.parent_body = associated_actor["parent_body"]
        self.parent_link = associated_actor["parent_link"]
        self.auto_rendering = associated_actor["auto_rendering"]
        intrinsic_param = associated_actor["intrinsic_param"].split(';')

        intrinsic_length = len(intrinsic_param)
        if intrinsic_length != 9:
            raise ValueError(
                f"Ill-formated intrinsic matrix {associated_actor['intrinsic_param']} for kube_bullet camera: we expect a 3x3 intrinsic calibaration matrix as vector of lentgh 9, given position has lenght: {intrinsic_length}")
        self.intrinsic_param = [int(intrinsic_elem) for intrinsic_elem in intrinsic_param]

        self.current_state = KubeBulletSpawnCameraActionState.IDLE
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
        if self.current_state == KubeBulletSpawnCameraActionState.IDLE:

            self.client.stub.SetupCamera(kube_bullet_grpc_pb2.CameraSetupRequest(
                command='spawn',
                camera_name=self.camera_name,
                parent_body=self.parent_body,
                parent_link=self.parent_link,
                auto_rendering=self.auto_rendering,
                intrinsic_param=self.intrinsic_param
            ))

            self.current_state = KubeBulletSpawnCameraActionState.REQUEST_SENT
            self.feedback_message = f"Request sent."  # pylint: disable= attribute-defined-outside-init
            return py_trees.common.Status.RUNNING
        elif self.current_state == KubeBulletSpawnCameraActionState.REQUEST_SENT:
            self.feedback_message = f"Success"  # pylint: disable= attribute-defined-outside-init
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE
