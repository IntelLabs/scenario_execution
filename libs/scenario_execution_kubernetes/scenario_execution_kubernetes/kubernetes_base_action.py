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

from kubernetes import client, config
from enum import Enum
import py_trees
import json
from scenario_execution.actions.base_action import BaseAction


class KubernetesBaseActionState(Enum):
    IDLE = 1
    REQUEST_SENT = 2
    FAILURE = 3


class KubernetesBaseAction(BaseAction):

    def __init__(self, within_cluster: bool):
        super().__init__()
        self.namespace = None
        self.within_cluster = within_cluster
        self.client = None
        self.current_state = KubernetesBaseActionState.IDLE
        self.current_request = None

    def setup(self, **kwargs):
        if self.within_cluster:
            config.load_incluster_config()
        else:
            config.load_kube_config()
        self.client = client.CoreV1Api()

    def execute(self, namespace: str):
        self.namespace = namespace

    def update(self) -> py_trees.common.Status:  # pylint: disable=too-many-return-statements
        if self.current_state == KubernetesBaseActionState.IDLE:
            self.current_request = self.kubernetes_call()
            self.current_state = KubernetesBaseActionState.REQUEST_SENT
            return py_trees.common.Status.RUNNING
        elif self.current_state == KubernetesBaseActionState.REQUEST_SENT:
            success = True
            if self.current_request.ready():
                if not self.current_request.successful():
                    try:
                        self.current_request.get()
                    except client.exceptions.ApiException as e:
                        message = ""
                        body = json.loads(e.body)
                        if "message" in body:
                            message = f", message: '{body['message']}'"
                        self.feedback_message = f"Failure! Reason: {e.reason} {message}"  # pylint: disable= attribute-defined-outside-init
                    success = False
            if success:
                return py_trees.common.Status.SUCCESS
            else:
                return py_trees.common.Status.FAILURE
        return py_trees.common.Status.FAILURE

    def kubernetes_call(self):
        # Use async_req = True, namespace=self.namespace
        raise NotImplementedError("Implement in derived action")
