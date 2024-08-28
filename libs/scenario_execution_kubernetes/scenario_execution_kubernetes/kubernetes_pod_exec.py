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
from kubernetes import client, config, stream
from enum import Enum
import re


class KubernetesPodExecState(Enum):
    IDLE = 1
    WAITING_FOR_LIST_RUNNING = 2
    POD_NAME_PRESENT = 3
    RUNNING = 4
    FAILURE = 5


class KubernetesPodExec(BaseAction):

    def __init__(self, within_cluster: bool):
        super().__init__()
        self.target = None
        self.namespace = None
        self.regex = None
        self.command = None
        self.within_cluster = within_cluster
        self.client = None
        self.reponse_queue = queue.Queue()
        self.current_state = KubernetesPodExecState.IDLE
        self.output_queue = queue.Queue()
        self.pod_list_request = None
        self.pod_name = None

    def setup(self, **kwargs):
        if self.within_cluster:
            config.load_incluster_config()
        else:
            config.load_kube_config()
        self.client = client.CoreV1Api()

        self.exec_thread = threading.Thread(target=self.pod_exec, daemon=True)

    def execute(self, target: str, command: list, regex: bool, namespace: str):
        self.target = target
        self.namespace = namespace
        self.command = command
        self.regex = regex
        if self.pod_list_request:
            self.pod_list_request.cancel()
        self.pod_name = None
        self.current_state = KubernetesPodExecState.IDLE

    def update(self) -> py_trees.common.Status:  # pylint: disable=too-many-return-statements
        if self.current_state == KubernetesPodExecState.IDLE:
            if self.regex:
                self.current_state = KubernetesPodExecState.WAITING_FOR_LIST_RUNNING
                self.feedback_message = f"Requesting list of pods in namespace '{self.namespace}'"  # pylint: disable= attribute-defined-outside-init
                self.pod_list_request = self.client.list_namespaced_pod(namespace=self.namespace, async_req=True)
                return py_trees.common.Status.RUNNING
            else:
                self.pod_name = self.target
                self.current_state = KubernetesPodExecState.POD_NAME_PRESENT

        if self.current_state == KubernetesPodExecState.WAITING_FOR_LIST_RUNNING:
            if not self.pod_list_request.ready():
                return py_trees.common.Status.RUNNING
            current_elements = []
            for i in self.pod_list_request.get().items:
                current_elements.append(i.metadata.name)

            found_element = None
            matched_elements = []
            for element in current_elements:
                if re.search(self.target, element):
                    matched_elements.append(element)
            if matched_elements:
                if len(matched_elements) > 1:
                    self.feedback_message = f"'{self.target}' regex identified more than one pod {', '.join(matched_elements)}. Only one element is supported!"  # pylint: disable= attribute-defined-outside-init
                    return py_trees.common.Status.FAILURE
                found_element = matched_elements[0]

            if found_element:
                self.pod_name = found_element
                self.current_state = KubernetesPodExecState.POD_NAME_PRESENT
            else:
                self.feedback_message = f"'{self.target}' not found in list of available pods (namespace: '{self.namespace}'). Available: {', '.join(current_elements)}'"  # pylint: disable= attribute-defined-outside-init
                return py_trees.common.Status.FAILURE

        if self.current_state == KubernetesPodExecState.POD_NAME_PRESENT:
            self.current_state = KubernetesPodExecState.RUNNING
            self.feedback_message = f"Executing on pod '{self.pod_name}': {self.command}..."  # pylint: disable= attribute-defined-outside-init
            self.exec_thread.start()
            return py_trees.common.Status.RUNNING
        elif self.current_state == KubernetesPodExecState.RUNNING:
            while not self.output_queue.empty():
                self.logger.debug(self.output_queue.get())
            try:
                response = self.reponse_queue.get_nowait()
                try:
                    if response.returncode == 0:
                        self.feedback_message = f"Execution successful."  # pylint: disable= attribute-defined-outside-init
                        return py_trees.common.Status.SUCCESS
                except ValueError:
                    self.feedback_message = f"Error while executing."  # pylint: disable= attribute-defined-outside-init
            except queue.Empty:
                return py_trees.common.Status.RUNNING

        return py_trees.common.Status.FAILURE

    def pod_exec(self):
        resp = stream.stream(self.client.connect_get_namespaced_pod_exec,
                             self.pod_name,
                             self.namespace,
                             command=self.command,
                             stderr=True, stdin=False,
                             stdout=True, tty=False,
                             _preload_content=False)

        while resp.is_open():
            resp.update(timeout=0.1)
            if resp.peek_stdout():
                self.output_queue.put(resp.read_stdout())
            if resp.peek_stderr():
                self.output_queue.put(resp.read_stderr())

        self.reponse_queue.put(resp)
