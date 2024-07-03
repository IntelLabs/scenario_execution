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

import os
import py_trees
from scenario_execution_gazebo.actions.utils import SpawnUtils
from scenario_execution.actions.base_action import BaseAction
from shutil import which
import tempfile


class GenerateGazeboWorld(BaseAction):

    def __init__(self, associated_actor, sdf_template: str, arguments: list):
        super().__init__()
        self.sdf_template = sdf_template
        self.spawn_utils = SpawnUtils(self.logger)

    def setup(self, **kwargs):
        if which("xacro") is None:
            raise ValueError("'xacro' not found.")
        if "input_dir" not in kwargs:
            raise ValueError("input_dir not defined.")
        input_dir = kwargs["input_dir"]

        if not os.path.isabs(self.sdf_template):
            self.sdf_template = os.path.join(input_dir, self.sdf_template)
        if not os.path.isfile(self.sdf_template):
            raise ValueError(f"SDF Template {self.sdf_template} not found.")
        self.tmp_file = tempfile.NamedTemporaryFile(suffix=".sdf")  # for testing, do not delete temp file: delete=False

    def execute(self, associated_actor, sdf_template: str, arguments: list):
        self.arguments_string = ""
        for elem in arguments:
            self.arguments_string += f'{elem["key"]}:={elem["value"]}'

    def update(self) -> py_trees.common.Status:
        world_sdf = self.spawn_utils.xacro_to_urdf(self.sdf_template, self.arguments_string)
        with open(self.tmp_file.name, 'w') as f:
            f.write(world_sdf.replace("\\", ""))
        self.set_associated_actor_variable("generated_gazebo_world_path", self.tmp_file.name)
        return py_trees.common.Status.SUCCESS
