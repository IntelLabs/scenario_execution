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

from scenario_execution.actions.base_action import ActionError
from scenario_execution.actions.run_process import RunProcess
from scenario_execution.scenario_execution_base import ScenarioExecutionConfig
import signal
import os


class RosLaunch(RunProcess):

    def execute(self, package_name: str, launch_file: str, arguments: list, wait_for_shutdown: bool, shutdown_timeout: float):  # pylint: disable=arguments-differ
        super().execute(None, wait_for_shutdown, shutdown_timeout, shutdown_signal=("", signal.SIGINT))

        if not package_name:
            if not os.path.isabs(launch_file):
                launch_file = os.path.join(ScenarioExecutionConfig().scenario_file_directory, launch_file)
            self.command = ["ros2", "launch", launch_file]
        else:
            self.command = ["ros2", "launch", package_name, launch_file]
        if isinstance(arguments, list):
            for arg in arguments:
                if 'key' not in arg or 'value' not in arg:
                    raise ActionError(f'Invalid argument: {arg}', action=self)
                if arg["key"] is not None:
                    self.command.append(f'{arg["key"]}:={arg["value"]}')
        self.logger.info(f'Command: {" ".join(self.command)}')
