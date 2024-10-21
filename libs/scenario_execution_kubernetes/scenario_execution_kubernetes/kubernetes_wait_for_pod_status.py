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
import re


class KubernetesWaitForPodStatus(BaseAction):

    def __init__(self, within_cluster: bool):
        super().__init__()
        self.target = None
        self.expected_status = None
        self.within_cluster = within_cluster
        self.regex = None
        self.client = None
        self.update_queue = queue.Queue()
        self.is_pod = False

    def setup(self, **kwargs):
        if self.within_cluster:
            config.load_incluster_config()
        else:
            config.load_kube_config()
        self.client = client.CoreV1Api()

    def execute(self, target: str, regex: bool, status: tuple, namespace: str):
        self.namespace = namespace
        self.target = target
        if not isinstance(status, tuple) or not isinstance(status[0], str):
            raise ValueError("Status expected to be enum.")
        self.expected_status = status[0]
        self.regex = regex
        self.is_pod = False
        self.monitoring_thread = threading.Thread(target=self.watch_pods, daemon=True)
        self.monitoring_thread.start()

    def update(self) -> py_trees.common.Status:
        while not self.update_queue.empty():
            item = self.update_queue.get()
            if len(item) != 2:
                return py_trees.common.Status.FAILURE
            if not self.is_pod:
                self.feedback_message = f"waiting for status of pod '{self.target}'."  # pylint: disable= attribute-defined-outside-init
            if not self.regex:
                if item[0] != self.target:
                    continue
            else:
                if not re.search(self.target, item[0]):
                    continue
            # self.feedback_message = f"The pod '{self.target}' is currently in '{item[1].lower()}' status."  # pylint: disable= attribute-defined-outside-init
            if item[1].lower() == self.expected_status:
                self.is_pod = True
                self.feedback_message = f"Pod '{item[0]}' changed to expected status '{item[1].lower()}'."  # pylint: disable= attribute-defined-outside-init
                return py_trees.common.Status.SUCCESS
            else:
                self.is_pod = True
                self.feedback_message = f"Pod '{item[0]}' changed to status '{item[1].lower()}', expected '{self.expected_status}'."  # pylint: disable= attribute-defined-outside-init
        return py_trees.common.Status.RUNNING

    def watch_pods(self):
        w = watch.Watch()
        try:
            # TODO: make use of send_initial_events=false in the future
            for event in w.stream(self.client.list_namespaced_pod, namespace=self.namespace):
                pod_name = event['object'].metadata.name
                pod_status = event['object'].status.phase
                self.update_queue.put((pod_name, pod_status))
        except ApiException as e:
            self.logger.error(f"Error accessing kubernetes: {e}")
            self.update_queue.put(())
