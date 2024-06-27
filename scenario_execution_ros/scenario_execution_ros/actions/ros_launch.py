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

from enum import Enum

from scenario_execution.actions.run_process import RunProcess
import signal


class RosLaunchActionState(Enum):
    """
    States for executing a ros bag recording
    """
    WAITING_FOR_TOPICS = 1
    RECORDING = 2
    FAILURE = 5


class RosLaunch(RunProcess):
    """
    Class to execute ros bag recording
    """

    def __init__(self, name, package_name: str, launch_file: str, arguments: list, wait_for_finished: bool, shutdown_timeout: float):
        super().__init__(name, None, wait_for_finished, shutdown_timeout, shutdown_signal=("", signal.SIGINT))
        self.package_name = package_name
        self.launch_file = launch_file
        self.arguments = arguments
        self.wait_for_finish = wait_for_finished
        self.command = None

    def setup(self, **kwargs):
        self.command = ["ros2", "launch", self.package_name, self.launch_file]

        for arg in self.arguments:
            if not arg["key"] or not arg["value"]:
                raise ValueError(f'Invalid ros argument key:{arg["key"]}, value:{arg["value"]}')
            self.command.append(f'{arg["key"]}:={arg["value"]}')

        self.logger.info(f'Command: {" ".join(self.command)}')
