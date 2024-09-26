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
import signal
import os

class RosLaunch(RunProcess):
    def __init__(self):
        super().__init__()
        self.input_dir = None
        
    def setup(self, **kwargs):
        if "input_dir" not in kwargs:
            raise ActionError("input_dir not defined.", action=self)
        self.input_dir = kwargs["input_dir"]
        
    def execute(self, package_name: str, launch_file: str, arguments: list, replace_relative_paths_in_args, wait_for_shutdown: bool, shutdown_timeout: float):  # pylint: disable=arguments-differ
        super().execute(None, wait_for_shutdown, shutdown_timeout, shutdown_signal=("", signal.SIGINT))
        self.command = ["ros2", "launch", package_name, launch_file]
        if isinstance(arguments, list):
            for arg in arguments:
                if 'key' not in arg or 'value' not in arg:
                    raise ActionError(f'Invalid argument: {arg}', action=self)
                if arg["key"] is not None:
                    value = arg["value"]
                    if replace_relative_paths_in_args:
                        if value.startswith('./') or value.startswith('../'):
                            value = os.path.abspath(os.path.join(self.input_dir, value))
                    self.command.append(f'{arg["key"]}:={value}')
        self.logger.info(f'Command: {" ".join(self.command)}')
