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

from scenario_execution.scenario_execution_base import ScenarioExecutionConfig

def get_scenario_file_directory():
    """
    Returns the directory where scenario files are stored.
    """
    return ScenarioExecutionConfig().scenario_file_directory

def get_output_directory():
    """
    Returns the output directory for scenario execution.
    """
    return ScenarioExecutionConfig().output_directory
