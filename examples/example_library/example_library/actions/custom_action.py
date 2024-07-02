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

""" Example for creating an action plugin for scenario execution """
import py_trees
from scenario_execution.actions.base_action import BaseAction


class CustomAction(BaseAction):

    def __init__(self, data: str):  # get action arguments, at the time of initialization
        super().__init__()

    def execute(self, data: str):  # get action arguments, at the time of execution (may got updated during scenario execution)
        self.data = data

    # Override the update function to define how the behavior is ticking.
    def update(self):
        print(f"Custom Action Triggered. Data: {self.data}")
        return py_trees.common.Status.SUCCESS
