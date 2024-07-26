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

import py_trees
from scenario_execution.actions.base_action import BaseAction
import queue
import threading
from kubernetes import client, config, watch
from kubernetes.client.rest import ApiException


class KubernetesWaitForNetworkPolicyStatus(BaseAction):

    def __init__(self, target: str, status: tuple, namespace: str, within_cluster: bool):
        super().__init__()
        self.target = target
        self.namespace = namespace
        if not isinstance(status, tuple) or not isinstance(status[0], str):
            raise ValueError("Status expected to be enum.")
        self.expected_status = status[0]
        self.within_cluster = within_cluster
        self.network_client = None
        self.update_queue = queue.Queue()

    def setup(self, **kwargs):
        if self.within_cluster:
            config.load_incluster_config()
        else:
            config.load_kube_config()
        self.k8s_client = client.api_client.ApiClient()
        self.network_client = client.NetworkingV1Api(self.k8s_client)

    def execute(self, target: str, status: tuple, namespace: str, within_cluster: bool):
        self.monitoring_thread = threading.Thread(target=self.watch_network, daemon=True)
        self.monitoring_thread.start()

    def update(self) -> py_trees.common.Status:
        while not self.update_queue.empty():
            item = self.update_queue.get()
            if len(item) != 2:
                return py_trees.common.Status.FAILURE

            if item[0] == self.target:
                if item[1].lower() == self.expected_status:
                    self.feedback_message = f"Expected status '{item[1].lower()}' found."  # pylint: disable= attribute-defined-outside-init
                    return py_trees.common.Status.SUCCESS
                else:
                    self.feedback_message = f"Status changed to '{item[1].lower()}', expected '{self.expected_status}'."  # pylint: disable= attribute-defined-outside-init
        return py_trees.common.Status.RUNNING

    def watch_network(self):
        w = watch.Watch()
        try:
            for event in w.stream(self.network_client.list_namespaced_network_policy, self.namespace):
                network_policy_name = event['object'].metadata.name
                event_type = event['type']
                self.update_queue.put((network_policy_name, event_type))
        except ApiException as e:
            self.logger.error(f"Error accessing kubernetes: {e}")
            self.update_queue.put(())
