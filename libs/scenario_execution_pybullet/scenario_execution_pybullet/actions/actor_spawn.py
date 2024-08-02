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

import py_trees
from scenario_execution.actions.base_action import BaseAction
import pybullet as p


class ActorSpawn(BaseAction):

    def __init__(self):
        super().__init__()
        self.spawn_pose = None
        self.model = None
        self.actor_id = None

    def execute(self, associated_actor, model: str, pose: dict):  # pylint: disable=arguments-differ
        self.spawn_pose = pose
        self.pos = [pose["position"]["x"], pose["position"]["y"], pose["position"]["z"]]
        self.orientation = p.getQuaternionFromEuler([pose["orientation"]["roll"], pose["orientation"]["pitch"], pose["orientation"]["yaw"]])
        self.model = model

    def update(self) -> py_trees.common.Status:
        self.actor_id = p.loadURDF(self.model, self.pos, self.orientation)
        self.set_associated_actor_variable("actor_id", self.actor_id)
        return py_trees.common.Status.SUCCESS
