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

""" Scenario execution plugin to run an external command """

import py_trees  # pylint: disable=import-error
from threading import Thread, Lock
import socket
import time


class OpenPort(py_trees.behaviour.Behaviour):
    """
    Open a port (and keep it open until shutdown)

    Args:
        command[str]: external command to execute
    """

    def __init__(self, name, port, address):
        super().__init__(name)
        self.port = port
        self.address = address
        self.thread = None
        self.shutdown_requested = Lock()

    def update(self) -> py_trees.common.Status:
        """
        return:
            py_trees.common.Status
        """
        if not self.thread:
            def signal_ready(shutdown_requested):
                with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                    s.bind((self.address, self.port))
                    s.listen()
                    while not shutdown_requested.locked():
                        time.sleep(1)
            self.thread = Thread(target=signal_ready, args=(self.shutdown_requested,))
            self.thread.start()
            self.feedback_message = f"Port {self.port} is open."  # pylint: disable= attribute-defined-outside-init
        # The status might be reported to early (as listen() within thread might take time)
        return py_trees.common.Status.SUCCESS

    def cleanup(self):
        """
        Cleanup on shutdown
        """
        self.shutdown_requested.acquire()
        self.thread.join()
