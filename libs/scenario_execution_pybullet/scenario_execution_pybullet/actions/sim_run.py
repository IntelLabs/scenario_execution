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
import math


class SimRun(BaseAction):

    def __init__(self):
        super().__init__()
        self.sim_steps_per_tick = 0

    def setup(self, **kwargs):
        self.logger = kwargs["logger"]

        try:
            tick_period: float = kwargs['tick_period']
        except KeyError as e:
            raise KeyError("didn't find 'tick_period' in setup's kwargs") from e
        if not math.isclose(240 % tick_period, 0., abs_tol=1e-4):
            raise ValueError(
                f"Scenario Execution Tick Period of {tick_period} is not compatible with PyBullet stepping. Please set step-duration to be a multiple of 1/240s")
        self.sim_steps_per_tick = round(240 * tick_period)
        self.logger.info(f"Forward simulation by {self.sim_steps_per_tick} step per scenario tick.")

    def update(self) -> py_trees.common.Status:
        for _ in range(self.sim_steps_per_tick):
            p.stepSimulation()
        return py_trees.common.Status.RUNNING
