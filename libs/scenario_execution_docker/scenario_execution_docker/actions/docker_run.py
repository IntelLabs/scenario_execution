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

import docker
import py_trees
from scenario_execution.actions.base_action import BaseAction, ActionError


class ContainerStatus(Enum):
    IDLE = 1
    RUNNING = 2
    DONE = 3


class DockerRun(BaseAction):
    """
    Run a container
    """

    def __init__(self, image: str, command: str, container_name: str,
                 detach: bool, environment: list, network: str,
                 privileged: bool, remove: bool, stream: bool,  volumes: list):
        super().__init__()
        self.image = image
        self.command = command
        self.container_name = container_name
        self.detach = detach
        self.environment = environment
        self.network = network
        self.privileged = privileged
        self.remove = remove
        self.stream = stream
        self.volumes = volumes

        self.client = None
        self.container = None
        self.current_state = ContainerStatus.IDLE

    def setup(self, **kwargs):
        # create docker client
        self.client = docker.from_env()
        # check docker image
        filterred_images = self.client.images.list(filters={'reference': self.image})
        if len(filterred_images) == 0:
            raise ActionError(f"Required docker image '{self.image}' does not exist.", action=self)

    def update(self) -> py_trees.common.Status:
        if self.current_state == ContainerStatus.IDLE:
            try:
                self.container = self.client.containers.run(
                    self.image,
                    command=self.command,
                    detach=self.detach,
                    environment=self.environment,
                    name=self.container_name,
                    network=self.network,
                    privileged=self.privileged,
                    stream=self.stream,
                    remove=self.remove,
                    user=os.getuid(),
                    group_add=[os.getgid()],
                    volumes=self.volumes)
            except docker.errors.APIError as e:
                self.feedback_message = f"Docker run failed: {e}"  # pylint: disable= attribute-defined-outside-init
                return py_trees.common.Status.FAILURE
            self.current_state = ContainerStatus.RUNNING
            self.feedback_message = f"Running docker container {self.image}"  # pylint: disable= attribute-defined-outside-init
            return py_trees.common.Status.RUNNING
        elif self.current_state == ContainerStatus.RUNNING:
            if self.stream and not self.detach:
                try:
                    log = next(self.container)
                    self.feedback_message = f"Running container {self.image} with output: {log.decode()}"  # pylint: disable= attribute-defined-outside-init
                except StopIteration:
                    self.current_state = ContainerStatus.DONE
                    self.feedback_message = f"Docker container {self.image} finished cleanly"  # pylint: disable= attribute-defined-outside-init
                    return py_trees.common.Status.SUCCESS
            elif self.detach:
                self.container.reload()
                if self.container:
                    res = self.container.status
                    self.feedback_message = f"Running container {self.image} with status {res}"  # pylint: disable= attribute-defined-outside-init
                    if res in ["removing", "exited"]:
                        self.current_state = ContainerStatus.DONE
                        self.feedback_message = f"Docker container {self.image} finished cleanly"  # pylint: disable= attribute-defined-outside-init
                        return py_trees.common.Status.SUCCESS
        elif self.current_state == ContainerStatus.DONE:
            self.feedback_message = f"Docker container {self.image} finished cleanly"  # pylint: disable= attribute-defined-outside-init
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING

    def shutdown(self):
        if self.container is None:
            return

        self.container.stop(timeout=0)
