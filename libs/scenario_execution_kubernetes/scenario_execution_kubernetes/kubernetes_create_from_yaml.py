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

from kubernetes import client, config, utils
from enum import Enum
import py_trees
import json
from scenario_execution.actions.base_action import BaseAction


class KubernetesCreateFromYamlActionState(Enum):
    IDLE = 1
    CREATION_REQUESTED = 2
    FAILURE = 3


class KubernetesCreateFromYaml(BaseAction):

    def __init__(self, namespace: str, yaml_file: str, within_cluster: bool):
        super().__init__()
        self.namespace = namespace
        self.yaml_file = yaml_file
        self.within_cluster = within_cluster
        self.client = None
        self.current_state = KubernetesCreateFromYamlActionState.IDLE
        self.current_request = None

    def setup(self, **kwargs):
        if self.within_cluster:
            config.load_incluster_config()
        else:
            config.load_kube_config()
        self.client = client.api_client.ApiClient()

    def update(self) -> py_trees.common.Status:  # pylint: disable=too-many-return-statements
        if self.current_state == KubernetesCreateFromYamlActionState.IDLE:
            try:
                self.current_request = utils.create_from_yaml(
                    self.client, self.yaml_file, verbose=False, namespace=self.namespace, async_req=True)
            except Exception as e:  # pylint: disable=broad-except
                self.feedback_message = f"Error while creating from yaml: {e}"
                return py_trees.common.Status.FAILURE
            self.current_state = KubernetesCreateFromYamlActionState.CREATION_REQUESTED
            self.feedback_message = f"Requested creation from yaml file '{self.yaml_file}' in namespace '{self.namespace}'"  # pylint: disable= attribute-defined-outside-init
            return py_trees.common.Status.RUNNING
        elif self.current_state == KubernetesCreateFromYamlActionState.CREATION_REQUESTED:
            success = True
            for reqs in self.current_request:
                for req in reqs:
                    if req.ready():
                        if not req.successful():
                            try:
                                req.get()
                            except client.exceptions.ApiException as e:
                                message = ""
                                body = json.loads(e.body)
                                if "message" in body:
                                    message = f", message: '{body['message']}'"
                                self.feedback_message = f"Failure! Reason: {e.reason} {message}"  # pylint: disable= attribute-defined-outside-init
                            success = False
                    else:
                        return py_trees.common.Status.RUNNING
            if success:
                return py_trees.common.Status.SUCCESS
            else:
                return py_trees.common.Status.FAILURE
        return py_trees.common.Status.FAILURE
