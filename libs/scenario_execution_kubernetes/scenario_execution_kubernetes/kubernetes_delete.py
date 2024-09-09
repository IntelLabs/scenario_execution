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
import re
from scenario_execution.actions.base_action import BaseAction


class KubernetesElementType(Enum):
    POD = 1
    NETWORK_POLICY = 2


class KubernetesDeleteActionState(Enum):
    IDLE = 1
    WAITING_FOR_LIST_RUNNING = 2
    DELETE_REQUESTED = 3
    WAIT_FOR_DELETION = 4
    FAILURE = 5


class KubernetesDelete(BaseAction):

    def __init__(self, target: str, namespace: str, element_type: tuple, grace_period: int, regex: bool, within_cluster: bool):
        super().__init__()
        self.target = target
        self.namespace = namespace
        if not isinstance(element_type, tuple) or not isinstance(element_type[0], str):
            raise ValueError("Element type expected to be enum.")
        self.element_type_str = element_type[0]
        if self.element_type_str == "pod":
            self.element_type = KubernetesElementType.POD
        elif self.element_type_str == "networkpolicy":
            self.element_type = KubernetesElementType.NETWORK_POLICY
        else:
            raise ValueError(f"element_type {element_type[0]} unknown.")
        self.grace_period = grace_period
        self.regex = regex
        self.within_cluster = within_cluster
        self.client = None
        self.current_state = KubernetesDeleteActionState.IDLE
        self.current_request = None

    def setup(self, **kwargs):
        if self.within_cluster:
            config.load_incluster_config()
        else:
            config.load_kube_config()

        self.client = client.CoreV1Api()
        self.k8s_client = client.api_client.ApiClient()
        self.network_client = client.NetworkingV1Api(self.k8s_client)

    def update(self) -> py_trees.common.Status:  # pylint: disable=too-many-return-statements
        if self.current_state == KubernetesDeleteActionState.IDLE:
            self.current_state = KubernetesDeleteActionState.WAITING_FOR_LIST_RUNNING
            self.feedback_message = f"Requesting list of elements of type {self.element_type_str} in namespace '{self.namespace}'"  # pylint: disable= attribute-defined-outside-init
            if self.element_type == KubernetesElementType.POD:
                self.current_request = self.client.list_namespaced_pod(
                    namespace=self.namespace, async_req=True)
            elif self.element_type == KubernetesElementType.NETWORK_POLICY:
                self.current_request = self.network_client.list_namespaced_network_policy(
                    namespace=self.namespace, async_req=True)
            return py_trees.common.Status.RUNNING
        elif self.current_state == KubernetesDeleteActionState.WAITING_FOR_LIST_RUNNING:
            if self.current_request.ready():
                current_elements = []
                for i in self.current_request.get().items:
                    current_elements.append(i.metadata.name)

                found_element = None
                if self.regex:
                    matched_elements = []
                    for element in current_elements:
                        if re.search(self.target, element):
                            matched_elements.append(element)
                    if matched_elements:
                        if len(matched_elements) > 1:
                            self.feedback_message = f"'{self.target}' regex identified more than one element of type {self.element_type_str} {', '.join(matched_elements)}. Only one element is supported!"  # pylint: disable= attribute-defined-outside-init
                            return py_trees.common.Status.FAILURE
                        found_element = matched_elements[0]
                else:
                    if self.target in current_elements:
                        found_element = self.target
                if found_element:
                    self.target = found_element
                    self.feedback_message = f"'{self.target}' found! Requesting deletion."  # pylint: disable= attribute-defined-outside-init
                    self.current_state = KubernetesDeleteActionState.DELETE_REQUESTED
                    if self.element_type == KubernetesElementType.POD:
                        self.current_request = self.client.delete_namespaced_pod(
                            found_element, namespace=self.namespace, grace_period_seconds=int(self.grace_period), async_req=True)
                    elif self.element_type == KubernetesElementType.NETWORK_POLICY:
                        self.current_request = self.network_client.delete_namespaced_network_policy(
                            found_element, namespace=self.namespace, grace_period_seconds=int(self.grace_period), async_req=True)
                    return py_trees.common.Status.RUNNING
                else:
                    self.feedback_message = f"'{self.target}' not found in list of current elements of type {self.element_type_str} (namespace: '{self.namespace}'). Available: {', '.join(current_elements)}'"  # pylint: disable= attribute-defined-outside-init
                    return py_trees.common.Status.FAILURE
            return py_trees.common.Status.RUNNING
        elif self.current_state == KubernetesDeleteActionState.DELETE_REQUESTED:
            if self.current_request.ready():
                if self.current_request.successful():
                    self.feedback_message = f"'{self.target}' deletion successfully requested!"  # pylint: disable= attribute-defined-outside-init
                    self.current_request = None
                    self.current_state = KubernetesDeleteActionState.WAIT_FOR_DELETION
                    if self.element_type == KubernetesElementType.POD:
                        self.current_request = self.client.list_namespaced_pod(
                            namespace=self.namespace, async_req=True)
                    elif self.element_type == KubernetesElementType.NETWORK_POLICY:
                        self.current_request = self.network_client.list_namespaced_network_policy(
                            namespace=self.namespace, async_req=True)
                    return py_trees.common.Status.RUNNING
                else:
                    self.feedback_message = f"Error while deleting '{self.target}' in namespace '{self.namespace}'."  # pylint: disable= attribute-defined-outside-init
            else:
                return py_trees.common.Status.RUNNING
        elif self.current_state == KubernetesDeleteActionState.WAIT_FOR_DELETION:
            if self.current_request.ready():
                if self.current_request.successful():
                    current_elements = []
                    for i in self.current_request.get().items:
                        current_elements.append(i.metadata.name)
                    if self.target not in current_elements:
                        self.feedback_message = f"successfully deleted '{self.target}' from namespace '{self.namespace}'."  # pylint: disable= attribute-defined-outside-init
                        return py_trees.common.Status.SUCCESS
                    else:
                        self.feedback_message = f"Waiting for deletion of '{self.target}' in namespace '{self.namespace}'."  # pylint: disable= attribute-defined-outside-init
                        if self.element_type == KubernetesElementType.POD:
                            self.current_request = self.client.list_namespaced_pod(
                                namespace=self.namespace, async_req=True)
                        elif self.element_type == KubernetesElementType.NETWORK_POLICY:
                            self.current_request = self.network_client.list_namespaced_network_policy(
                                namespace=self.namespace, async_req=True)
                        return py_trees.common.Status.RUNNING
                else:
                    self.feedback_message = f"Error while deleting '{self.target}' in namespace '{self.namespace}'."  # pylint: disable= attribute-defined-outside-init
            else:
                return py_trees.common.Status.RUNNING

        else:
            self.feedback_message = f"Error while requesting list of current elements of type {self.element_type_str} (after deletion request)."  # pylint: disable= attribute-defined-outside-init

        return py_trees.common.Status.FAILURE
