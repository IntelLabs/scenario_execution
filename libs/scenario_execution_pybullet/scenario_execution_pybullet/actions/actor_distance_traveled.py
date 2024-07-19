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
from math import sqrt


class ActorDistanceTraveled(BaseAction):

    def __init__(self):
        super().__init__()
        self.logger = None
        self.sim_steps_per_tick = None
        self.distance_traveled = None
        self.distance_expected = None
        self.previous_pos = None

    def setup(self, **kwargs):
        self.logger = kwargs["logger"]

    def execute(self, associated_actor, distance: float):  # pylint: disable=arguments-differ
        self.actor_id = self.get_associated_actor_variable("actor_id")
        self.distance_expected = distance

    def update(self) -> py_trees.common.Status:
        pos, _ = p.getBasePositionAndOrientation(self.actor_id)

        if not isinstance(pos, list) and len(pos) != 3:
            self.feedback_message = f"Unexpected position received from simulation: {pos}"  # pylint: disable= attribute-defined-outside-init
            return py_trees.common.Status.FAILURE

        if self.distance_traveled is not None:
            d_increment = sqrt((pos[0] - self.previous_pos[0]) * (pos[0] - self.previous_pos[0]) +
                               (pos[1] - self.previous_pos[1]) * (pos[1] - self.previous_pos[1]))
            self.distance_traveled = self.distance_traveled + d_increment
        else:
            self.distance_traveled = 0.
        self.previous_pos = pos

        if self.distance_traveled >= self.distance_expected:
            self.feedback_message = f"expected traveled distance reached: {float(self.distance_expected):.3}"  # pylint: disable= attribute-defined-outside-init
            return py_trees.common.Status.SUCCESS
        else:
            self.feedback_message = f"distance traveled: {float(self.distance_traveled):.3} < {float(self.distance_expected):.3}"  # pylint: disable= attribute-defined-outside-init
        return py_trees.common.Status.RUNNING
