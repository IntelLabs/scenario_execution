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
import pybullet_data


class SimInitialize(BaseAction):

    def __init__(self):
        super().__init__()
        self.logger = None
        self.world = None

    def setup(self, **kwargs):
        self.logger = kwargs["logger"]

    def execute(self, associated_actor, world: str):  # pylint: disable=arguments-differ
        self.world = world

    def update(self) -> py_trees.common.Status:
        p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.8)
        p.loadURDF(self.world)
        return py_trees.common.Status.SUCCESS
