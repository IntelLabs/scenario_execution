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

""" Scenario execution plugin to wait for ports """

import py_trees  # pylint: disable=import-error
from threading import Thread, Lock
import socket
import time
from collections import deque


class WaitForOpenPorts(py_trees.behaviour.Behaviour):
    """
    Wait for network ports to open

    Args:
        targets[str]: list of targets (host:port)
    """

    def __init__(self, name, targets):
        super().__init__(name)
        self.threads = []
        self.shutdown_requested = Lock()
        self.targets = targets
        self.targets_done = deque()

    def setup(self, **kwargs):
        """
        parse
        """
        tmp = self.targets.split(';')
        self.targets = []
        for elem in tmp:
            pair = elem.split(':')
            if len(pair) != 2:
                raise ValueError("Invalid pair")
            self.targets.append((pair[0], pair[1]))
        if len(self.targets) == 0:
            raise ValueError(
                f"Targets invalid. Expected format: <host>:<port>;<host>:<port>;.. Found {tmp}")

    def update(self) -> py_trees.common.Status:
        """
        return:
            py_trees.common.Status
        """
        if not self.threads:
            def wait_for_port(host, port, shutdown_requested):
                while not shutdown_requested.locked():
                    try:
                        with socket.create_connection((host, port)):
                            return True
                    except OSError:
                        time.sleep(1.)
                return False

            for target in self.targets:
                thread = Thread(target=wait_for_port, args=(
                    target[0], target[1], self.shutdown_requested), name=f"{target[0]};{target[1]}")
                thread.start()
                thread.done = False
                self.threads.append(thread)
            return py_trees.common.Status.RUNNING

        for t in self.threads:
            if not t.is_alive():
                t.done = True
        self.threads = [t for t in self.threads if not t.done]
        running_desc = ";".join([t.name for t in self.threads])
        if self.threads:
            self.feedback_message = f"Waiting for {running_desc}"  # pylint: disable= attribute-defined-outside-init
            return py_trees.common.Status.RUNNING
        else:
            self.feedback_message = "All ports found."  # pylint: disable= attribute-defined-outside-init
            return py_trees.common.Status.SUCCESS

    def cleanup(self):
        """
        Cleanup on shutdown
        """
        self.shutdown_requested.acquire()
        for t in self.threads:
            t.join()
