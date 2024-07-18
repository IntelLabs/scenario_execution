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

import subprocess  # nosec B404
from enum import Enum

from transforms3d.taitbryan import euler2quat
from std_msgs.msg import String

from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.logging import get_logger
from rclpy.node import Node
import py_trees
from scenario_execution.actions.base_action import BaseAction

import pybullet as p

class SpawnActionState(Enum):
    """
    States for executing a spawn-entity in gazebo
    """
    WAITING_FOR_TOPIC = 1
    MODEL_AVAILABLE = 2
    WAITING_FOR_RESPONSE = 3
    DONE = 4
    FAILURE = 5


class ActorSpawn(BaseAction):
    """
    Class to spawn an entity into simulation

    """

    def __init__(self):
        """
        init
        """
        super().__init__(resolve_variable_reference_arguments_in_execute=False)
        self.current_state = SpawnActionState.WAITING_FOR_TOPIC
        # self.entity_name = associated_actor["name"]
        # self.entity_model = model
        self.logger = None
        self.actor_id = None

    def setup(self, **kwargs):
        self.logger = get_logger(self.name)

    def execute(self, associated_actor, model: str, pose: dict): # pylint: disable=arguments-differ
        # if self.entity_model != model:
        #     raise ValueError("Runtime change of model not supported.")
        self.spawn_pose = pose
        self.pos = [pose["position"]["x"], pose["position"]["y"], pose["position"]["z"]]
        self.orientation = p.getQuaternionFromEuler([pose["orientation"]["roll"],pose["orientation"]["pitch"],pose["orientation"]["yaw"]])
        self.model = model

    def update(self) -> py_trees.common.Status:
        """
        Send request
        """
        # if self.current_state == SpawnActionState.WAITING_FOR_TOPIC:
        
        self.actor_id = p.loadURDF(self.model, self.pos, self.orientation)
        self.set_associated_actor_variable("actor_id", self.actor_id)
        return py_trees.common.Status.SUCCESS
        # elif self.current_state == SpawnActionState.MODEL_AVAILABLE:
        #     try:
        #         self.set_command(self.sdf)
        #         self.current_state = SpawnActionState.WAITING_FOR_RESPONSE
        #         return super().update()
        #     except ValueError as e:
        #         self.feedback_message = str(e)  # pylint: disable= attribute-defined-outside-init
        #         self.current_state = SpawnActionState.FAILURE
        #         return py_trees.common.Status.FAILURE
        # else:
        #     return super().update()
