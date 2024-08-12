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


class KubernetesPodExecState(Enum):
    IDLE = 1
    RUNNING = 2
    FAILURE = 3


class KubernetesPodExec(BaseAction):

    def __init__(self, target: str, command: list, namespace: str, within_cluster: bool):
        super().__init__()
        self.target = target
        self.namespace = namespace
        self.command = command
        self.within_cluster = within_cluster
        self.client = None
        self.reponse_queue = queue.Queue()
        self.current_state = KubernetesPodExecState.IDLE
        self.output_queue = queue.Queue()

    def setup(self, **kwargs):
        if self.within_cluster:
            config.load_incluster_config()
        else:
            config.load_kube_config()
        self.client = client.CoreV1Api()

        self.exec_thread = threading.Thread(target=self.pod_exec, daemon=True)

    def execute(self, target: str, command: list, namespace: str, within_cluster: bool):
        if within_cluster != self.within_cluster:
            raise ValueError("parameter 'within_cluster' is not allowed to change since initialization.")
        self.target = target
        self.namespace = namespace
        self.command = command
        self.current_state = KubernetesPodExecState.IDLE

    def update(self) -> py_trees.common.Status:
        if self.current_state == KubernetesPodExecState.IDLE:
            self.current_state = KubernetesPodExecState.RUNNING
            self.feedback_message = f"Executing on pod '{self.target}': {self.command}..."  # pylint: disable= attribute-defined-outside-init
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
                             self.target,
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
