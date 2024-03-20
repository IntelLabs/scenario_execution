#!/usr/bin/env python
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

"""
Run scenario execution
"""
import os

from scenario_execution_control.application_runner import ApplicationRunner  # pylint: disable=relative-import


class ScenarioExecutionRunner(ApplicationRunner):
    """
    Executes scenario execution
    """

    def __init__(self, status_updated_fct, log_fct):  # pylint: disable=too-many-arguments
        super(ScenarioExecutionRunner, self).__init__(
            status_updated_fct,
            log_fct,
            "Executing scenario ")

    def execute_scenario(self, scenario_file, output_dir):
        """
        Executes scenario
        """
        cmdline = ["ros2", "run", "scenario_execution", "scenario_execution", "--ros-args", "-p", f"scenario:={scenario_file}", output_dir]

        return self.execute(cmdline, env=os.environ)
