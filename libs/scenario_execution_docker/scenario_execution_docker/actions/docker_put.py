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


class DockerPut(BaseAction):
    """
    Copy a file or folder from the local filesystem into a running container
    """

    def __init__(self, container: str, source_path: str, target_path: str):
        super().__init__()
        self.container = container
        self.source_path = source_path
        self.target_path = target_path

        self.container_object = None
        self.current_state = CopyStatus.IDLE
        self.client = None
        self.tar = None

    def setup(self, **kwargs):
        # create docker client
        self.client = docker.from_env()

        # check if source path exists
        if not os.path.exists(self.source_path):
            raise ActionError(f"The given source path {self.source_path} does not exist", action=self)

    def update(self) -> py_trees.common.Status:
        if self.current_state == CopyStatus.IDLE:
            try:
                self.container_object = self.client.containers.get(self.container)
                self.current_state = CopyStatus.FOUND_CONTAINER
            except docker.errors.APIError as e:
                self.feedback_message = f"Docker container {self.container} not yet running {e}"  # pylint: disable= attribute-defined-outside-init
                return py_trees.common.Status.RUNNING

        elif self.current_state == CopyStatus.FOUND_CONTAINER:
            self.tar = tempfile.NamedTemporaryFile(suffix=".tar")
            try:
                with tarfile.open(self.tar.name, 'w:') as tar:
                    tar.add(
                        self.source_path,
                        arcname=os.path.basename(self.source_path))
                self.current_state = CopyStatus.COPYING
            except tarfile.ReadError as e:
                self.feedback_message = f"Compressing data to a tar file from path {self.source_path} failed: {e}"  # pylint: disable= attribute-defined-outside-init
                return py_trees.common.Status.FAILURE
        elif self.current_state == CopyStatus.COPYING:
            success = self.container_object.put_archive(
                path=self.target_path,
                data=self.tar
            )
            if success:
                self.current_state = CopyStatus.DONE
                self.feedback_message = f"Copying data from path {self.source_path} to {self.target_path} inside container {self.container}"  # pylint: disable= attribute-defined-outside-init
            else:
                self.feedback_message = f"Copying data from path {self.source_path} to {self.target_path} inside container {self.container} failed: {e}"  # pylint: disable= attribute-defined-outside-init
                return py_trees.common.Status.FAILURE

        elif self.current_state == CopyStatus.DONE:
            self.feedback_message = f"Finished copying data from path {self.source_path} to {self.target_path} inside container {self.container}"  # pylint: disable= attribute-defined-outside-init
            return py_trees.common.Status.SUCCESS

        return py_trees.common.Status.RUNNING
