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
from scenario_execution.actions.base_action import BaseAction, ActionError

import docker
import tempfile
import tarfile


class GenerateFloorplan(BaseAction):
    def __init__(self, associated_actor, file_path: str):
        super().__init__()
        self.file_path = file_path
        self.tmp_dir = None
        self.client = None
        self.floorplan_name = None
        self.floorplan_tarball = None

    def setup(self, **kwargs):
        self.tmp_dir = tempfile.TemporaryDirectory(delete=None)
        self.output_dir = self.tmp_dir.name
        # self.output_dir = tempfile.mkdtemp() # for testing: does not remove directory afterwards

        if "input_dir" not in kwargs:
            raise ActionError("input_dir not defined.", action=self)
        input_dir = kwargs["input_dir"]
        # check docker image
        self.client = docker.from_env()
        image_name = 'floorplan:latest'
        filterred_images = self.client.images.list(filters={'reference': image_name})
        if len(filterred_images) == 0:
            raise ActionError(f"Required docker image '{image_name}' does not exist.", action=self)

        # check files
        self.input_dir = os.path.abspath(input_dir)
        if not os.path.isabs(self.file_path):
            self.file_path = os.path.join(os.path.abspath(input_dir), self.file_path)
        if not os.path.isfile(self.file_path):
            raise ActionError(f"Floorplan file {self.file_path} not found.", action=self)
        self.floorplan_name = os.path.splitext(os.path.basename(self.file_path))[0]
        srcname = os.path.basename(self.file_path)
        self.floorplan_tarball = tempfile.NamedTemporaryFile(suffix=".tar")
        with tarfile.open(self.floorplan_tarball.name, mode='w') as input_tar:
            input_tar.add(self.file_path, arcname=os.path.basename(self.file_path))

    def update(self) -> py_trees.common.Status:
        self.feedback_message = f"Generating meshes..."  # pylint: disable= attribute-defined-outside-init
        try:
            container = self.client.containers.run("floorplan:latest",
                                                        command=f'/bin/sh -c "sleep 60"',
                                                        detach=True,
                                                        remove=True,
                                                        user=os.getuid(),
                                                        group_add=[os.getgid()])
            with open(self.floorplan_tarball.name, 'rb') as f:
                container.put_archive('/usr/src/app/models/', f.read())
            # TODO: exec_run currently blocks until finished.
            exit_code, output = container.exec_run(
                f'blender --background --python exsce_floorplan/exsce_floorplan.py --python-use-system-env -- ../models/{os.path.basename(self.file_path)}')
        except docker.errors.APIError as e:
            self.feedback_message = f"Generating meshes failed: {e}"  # pylint: disable= attribute-defined-outside-init
            return py_trees.common.Status.FAILURE
        if exit_code != 0:
            self.feedback_message = f"Error during generation: output: {output.decode()}"  # pylint: disable= attribute-defined-outside-init
            return py_trees.common.Status.FAILURE
        result_data, _ = container.get_archive('/usr/src/app/output/')
        output_tar = tempfile.NamedTemporaryFile(suffix=".tar")
        with open(output_tar.name, 'wb') as f:
            for chunk in result_data:
                f.write(chunk)
        with tarfile.open(output_tar.name, 'r:') as tar:
            tar.extractall(self.output_dir)
        output_path = os.path.join(self.output_dir, "output", self.floorplan_name)
        mesh_file = os.path.join(output_path, "mesh", self.floorplan_name + ".stl")
        map_file = os.path.join(output_path, "map", self.floorplan_name + ".yaml")
        if os.path.isfile(mesh_file) and os.path.isfile(map_file):
            self.feedback_message = f"Meshes generated."  # pylint: disable= attribute-defined-outside-init
            self.set_associated_actor_variable("generated_floorplan_mesh_path", mesh_file)
            self.set_associated_actor_variable("generated_floorplan_map_path", map_file)
            self.set_associated_actor_variable("goal_poses", [
                [{'position': {'x': 4.3, 'y': 9.8, 'z': 0}, 'orientation': {'roll': 0, 'pitch': 0, 'yaw': 0}}, {'position': {'x': 4.3, 'y': 0.7, 'z': 0}, 'orientation': {'roll': 0, 'pitch': 0, 'yaw': 0}}],
                [{'position': {'x': 4.3, 'y': 0.7, 'z': 0}, 'orientation': {'roll': 0, 'pitch': 0, 'yaw': 0}}, {'position': {'x': 4.3, 'y': 9.8, 'z': 0}, 'orientation': {'roll': 0, 'pitch': 0, 'yaw': 0}}]
            ])
            # self.set_associated_actor_variable("goal_poses", [{'position': {'x': 4.3, 'y': 9.8, 'z': 0}, 'orientation': {'roll': 0, 'pitch': 0, 'yaw': 0}}, {'position': {'x': 4.3, 'y': 0.7, 'z': 0}, 'orientation': {'roll': 0, 'pitch': 0, 'yaw': 0}}, {'position': {'x': 0.7, 'y': 0.7, 'z': 0}, 'orientation': {'roll': 0, 'pitch': 0, 'yaw': 0}}])
                                                              
                                                              
        # robot.init_nav2(initial_pose: pose_3d(position: position_3d(x: 0.7m, y: 9.8m)))
        # robot.nav_through_poses([
        #     pose_3d(position_3d(x: 4.3m, y: 9.8m)),
        #     pose_3d(position_3d(x: 4.3m, y: 0.7m)),
        #     pose_3d(position_3d(x: 0.7m, y: 0.7m))
            
            
            return py_trees.common.Status.SUCCESS
        else:
            self.logger.error(f"Did not find required files: mesh: {mesh_file}, map: {map_file}")
            return py_trees.common.Status.FAILURE
