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


class KubernetesPatchNetworkPolicyState(Enum):
    IDLE = 1
    REQUEST_SENT = 2
    FAILURE = 3


class KubernetesPatchNetworkPolicy(BaseAction):

    def __init__(self, namespace: str, target: str, network_enabled: bool, match_label: tuple, within_cluster: bool):
        super().__init__()
        self.namespace = namespace
        self.target = target
        self.network_enabled = network_enabled
        self.within_cluster = within_cluster
        if not isinstance(match_label, dict) or not "key" in match_label or not "value" in match_label:
            raise ValueError("match_label expected to be key-value pair.")
        self.match_label = match_label
        self.client = None
        self.current_state = KubernetesPatchNetworkPolicyState.IDLE
        self.current_request = None

    def setup(self, **kwargs):
        if self.within_cluster:
            config.load_incluster_config()
        else:
            config.load_kube_config()
        self.client = client.api_client.ApiClient()
        self.network_client = client.NetworkingV1Api(self.client)

    def update(self) -> py_trees.common.Status:  # pylint: disable=too-many-return-statements
        if self.current_state == KubernetesPatchNetworkPolicyState.IDLE:
            self.current_request = self.network_client.patch_namespaced_network_policy(self.target, body=self.get_network_policy(
                policy_name=self.target, enable=self.network_enabled, match_label=self.match_label), namespace=self.namespace, async_req=True)
            self.current_state = KubernetesPatchNetworkPolicyState.REQUEST_SENT
            self.feedback_message = f"Requested patching '{self.target}' in namespace '{self.namespace}'"  # pylint: disable= attribute-defined-outside-init
            return py_trees.common.Status.RUNNING
        elif self.current_state == KubernetesPatchNetworkPolicyState.REQUEST_SENT:
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

    def get_network_policy(self, policy_name, match_label, enable):
        body = client.V1NetworkPolicy()
        body.metadata = client.V1ObjectMeta(name=f"{policy_name}")
        body.spec = client.V1NetworkPolicySpec(pod_selector=client.V1LabelSelector(match_labels={match_label["key"]: match_label["value"]}))
        if enable:
            body.spec.egress = [client.V1NetworkPolicyEgressRule()]
            body.spec.ingress = [client.V1NetworkPolicyIngressRule()]
        else:
            body.spec.egress = []
            body.spec.ingress = []
        return body
