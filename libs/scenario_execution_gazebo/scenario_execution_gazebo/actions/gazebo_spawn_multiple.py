# Copyright (C) 2025 Frederik Pasch
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

from .gazebo_spawn_actor import GazeboSpawnActor
from scenario_execution.actions.base_action import ActionError
from scenario_execution.actions.base_action_subtree import BaseActionSubtree

class GazeboSpawnMultiple(BaseActionSubtree):

    def __init__(self, entities: list, world_name: str):
        super().__init__()
        self.entities = entities
        self.world_name = world_name

    def get_execution_args(self, child):
        if child not in self.children:
            raise ActionError(
                f"Child {child} is not part of this GazeboSpawnMultiple action!", action=self)
        idx = self.children.index(child)
        return {"associated_actor": {'name': self.entities[idx]["entity_name"]},
                "spawn_pose": self.entities[idx]["spawn_pose"],
                "world_name": self.world_name}

    def create_subtree(self):
        for entity in self.entities:
            spawn_action = GazeboSpawnActor(
                associated_actor={'name': entity["entity_name"]},
                xacro_arguments=entity["xacro_arguments"],
                model=entity["model"])
            spawn_action._set_base_properities(   # pylint: disable=protected-access
                self.name, None, self.logger)
            self.add_child(spawn_action)
