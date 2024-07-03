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
from enum import Enum

import py_trees
from scenario_execution.actions.base_action import BaseAction

import docker
import tempfile


class GenerateFloorplanStatus(Enum):
    IDLE = 1
    DOCKER_RUNNING = 2
    DONE = 4
    FAILURE = 5


class GenerateFloorplan(BaseAction):
    def __init__(self, associated_actor, file_path: str):
        super().__init__()
        self.file_path = file_path
        self.tmp_dir = None
        self.container = None
        self.client = None
        self.floorplan_name = None
        self.current_state = GenerateFloorplanStatus.IDLE

    def setup(self, **kwargs):
        self.tmp_dir = tempfile.TemporaryDirectory()
        self.output_dir = self.tmp_dir.name
        # self.output_dir = tempfile.mkdtemp() # for testing: does not remove directory afterwards

        if "input_dir" not in kwargs:
            raise ValueError("input_dir not defined.")
        input_dir = kwargs["input_dir"]
        # check docker image
        self.client = docker.from_env()
        image_name = 'floorplan:latest'
        filterred_images = self.client.images.list(filters={'reference': image_name})
        if len(filterred_images) == 0:
            raise ValueError(f"Required docker image '{image_name}' does not exist.")

        # check files
        if not os.path.isabs(self.file_path):
            self.file_path = os.path.join(input_dir, self.file_path)
        if not os.path.isfile(self.file_path):
            raise ValueError(f"Floorplan file {self.file_path} not found.")
        self.floorplan_name = os.path.splitext(os.path.basename(self.file_path))[0]

    def update(self) -> py_trees.common.Status:
        if self.current_state == GenerateFloorplanStatus.IDLE:
            model_dir = os.path.dirname(os.path.abspath(self.file_path))
            try:
                self.container = self.client.containers.run("floorplan:latest",
                                                            command="blender --background --python exsce_floorplan/exsce_floorplan.py --python-use-system-env -- ../models/" +
                                                            os.path.basename(self.file_path),
                                                            detach=True,
                                                            remove=True,
                                                            user=os.getuid(),
                                                            group_add=[os.getgid()],
                                                            volumes={
                                                                model_dir: {
                                                                    "bind": "/usr/src/app/models",
                                                                    "mode": "ro"},
                                                                self.output_dir: {
                                                                    "bind": "/usr/src/app/output",
                                                                    "mode": "rw"},
                                                            })
            except docker.errors.APIError as e:
                self.feedback_message = f"Generating meshes failed: {e}"  # pylint: disable= attribute-defined-outside-init
                return py_trees.common.Status.FAILURE
            self.current_state = GenerateFloorplanStatus.DOCKER_RUNNING
            self.feedback_message = f"Generating meshes..."  # pylint: disable= attribute-defined-outside-init
            return py_trees.common.Status.RUNNING
        elif self.current_state == GenerateFloorplanStatus.DOCKER_RUNNING:
            self.container.reload()
            if self.container:
                res = self.container.status
            if res in ["removing", "exited"]:
                output_path = os.path.join(self.output_dir, self.floorplan_name)
                mesh_file = os.path.join(output_path, "mesh", self.floorplan_name + ".stl")
                map_file = os.path.join(output_path, "map", self.floorplan_name + ".yaml")
                if os.path.isfile(mesh_file) and os.path.isfile(map_file):
                    self.feedback_message = f"Meshes generated."  # pylint: disable= attribute-defined-outside-init
                    self.set_associated_actor_variable("generated_floorplan_mesh_path", mesh_file)
                    self.set_associated_actor_variable("generated_floorplan_map_path", map_file)
                    self.current_state = GenerateFloorplanStatus.DONE
                    return py_trees.common.Status.SUCCESS
                else:
                    self.logger.error(f"Did not find required files: mesh: {mesh_file}, map: {map_file}")
                    return py_trees.common.Status.FAILURE
            return py_trees.common.Status.RUNNING

        return py_trees.common.Status.FAILURE
