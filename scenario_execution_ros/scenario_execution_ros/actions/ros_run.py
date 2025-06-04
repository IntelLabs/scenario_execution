# Copyright (C) 2025 Frederik Pasch
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

from scenario_execution.actions.run_process import RunProcess
import signal


class RosRun(RunProcess):

    def execute(self, package_name: str, executable_name: str, wait_for_shutdown: bool, shutdown_timeout: float):  # pylint: disable=arguments-differ,arguments-renamed
        super().execute(None, wait_for_shutdown, shutdown_timeout, shutdown_signal=("", signal.SIGINT))
        self.command = ["ros2", "run", package_name, executable_name]
        self.logger.info(f'Command: {" ".join(self.command)}')
