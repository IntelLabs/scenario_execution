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


class ActorSetJointMotorControl(BaseAction):

    def __init__(self):
        super().__init__()
        self.target_velocity = None
        self.force = None

    def execute(self, associated_actor, target_velocity: float, force: float):  # pylint: disable=arguments-differ
        self.actor_id = self.get_associated_actor_variable("actor_id")
        self.target_velocity = target_velocity
        self.force = force

    def update(self) -> py_trees.common.Status:
        num_joints = p.getNumJoints(self.actor_id)
        self.feedback_message = f"Set velocity {self.target_velocity}m/s, force {self.force}'"  # pylint: disable= attribute-defined-outside-init
        for i in range(num_joints):
            p.setJointMotorControl(self.actor_id, i, p.VELOCITY_CONTROL, self.target_velocity, 1)

        return py_trees.common.Status.RUNNING
