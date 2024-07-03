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
    WAITING_FOR_TOPICS = 1
    RECORDING = 2
    FAILURE = 5


class RosLaunch(RunProcess):

    def __init__(self, package_name: str, launch_file: str, arguments: list, wait_for_shutdown: bool, shutdown_timeout: float):
        super().__init__(None, wait_for_shutdown, shutdown_timeout, shutdown_signal=("", signal.SIGINT))

    def execute(self, package_name: str, launch_file: str, arguments: list, wait_for_shutdown: bool, shutdown_timeout: float):  # pylint: disable=arguments-differ
        super().execute(None, wait_for_shutdown, shutdown_timeout, shutdown_signal=("", signal.SIGINT))
        self.command = ["ros2", "launch", package_name, launch_file]
        for arg in arguments:
            if not arg["key"] or not arg["value"]:
                raise ValueError(f'Invalid argument key:{arg["key"]}, value:{arg["value"]}')
            self.command.append(f'{arg["key"]}:={arg["value"]}')
        self.logger.info(f'Command: {" ".join(self.command)}')
