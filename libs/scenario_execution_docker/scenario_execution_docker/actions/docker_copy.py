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

from enum import Enum

import os
import docker
import tempfile
import tarfile
import py_trees
from scenario_execution.actions.base_action import BaseAction, ActionError


class CopyStatus(Enum):
    IDLE = 1
    FOUND_CONTAINER = 2
    COPYING = 3
    DONE = 4


class DockerCopy(BaseAction):
    """
    Copy a file or folder from a running container
    """

    def __init__(self, container: str, file_path: str):
        super().__init__()
        self.container = container
        self.file_path = file_path

        self.container_object = None
        self.current_state = CopyStatus.IDLE
        self.tmp_dir = None
        self.output_dir = None
        self.client = None
        self.result_data = None

    def setup(self, **kwargs):
        # create docker client
        self.client = docker.from_env()

        # check output_dir
        if "output_dir" not in kwargs:
            raise ActionError("output_dir not defined.", action=self)

        if kwargs['output_dir']:
            if not os.path.exists(kwargs['output_dir']):
                raise ActionError(f"Specified destination dir '{kwargs['output_dir']}' does not exist", action=self)
            self.output_dir = kwargs['output_dir']

    def update(self) -> py_trees.common.Status:
        if self.current_state == CopyStatus.IDLE:
            try:
                self.container_object = self.client.containers.get(self.container)
                self.current_state = CopyStatus.FOUND_CONTAINER
            except docker.errors.APIError as e:
                self.feedback_message = f"Docker container {self.container} not yet running {e}"  # pylint: disable= attribute-defined-outside-init
                return py_trees.common.Status.RUNNING

        elif self.current_state == CopyStatus.FOUND_CONTAINER:
            try:
                self.result_data, _ = self.container_object.get_archive(
                    path=self.file_path)
                self.current_state = CopyStatus.COPYING
                self.feedback_message = f"Copying data from path {self.file_path} in container {self.container} to {self.output_dir}"  # pylint: disable= attribute-defined-outside-init
            except docker.errors.APIError as e:
                self.feedback_message = f"Copying of data from path {self.file_path} failed: {e}"  # pylint: disable= attribute-defined-outside-init
                return py_trees.common.Status.FAILURE
        elif self.current_state == CopyStatus.COPYING:
            output_tar = tempfile.NamedTemporaryFile(suffix=".tar")
            try:
                with open(output_tar.name, 'wb') as f:
                    for chunk in self.result_data:
                        f.write(chunk)
                with tarfile.open(output_tar.name, 'r') as tar:
                    tar.extractall(self.output_dir)
                self.current_state = CopyStatus.DONE
            except tarfile.ReadError as e:
                self.feedback_message = f"Copying of data from path {self.file_path} failed: {e}"  # pylint: disable= attribute-defined-outside-init
                return py_trees.common.Status.FAILURE

        elif self.current_state == CopyStatus.DONE:
            self.feedback_message = f"Finished copying of data from path {self.file_path} to {self.output_dir}"  # pylint: disable= attribute-defined-outside-init
            return py_trees.common.Status.SUCCESS

        return py_trees.common.Status.RUNNING
