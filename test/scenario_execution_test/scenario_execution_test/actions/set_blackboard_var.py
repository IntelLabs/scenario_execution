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

import py_trees
from py_trees.common import Status
from scenario_execution.actions.base_action import BaseAction


class SetBlackboardVariable(BaseAction):

    def execute(self, variable_name: str, variable_value):
        self.variable_name = variable_name
        self.variable_value = variable_value
        self.get_blackboard_client().register_key(self.variable_name, access=py_trees.common.Access.WRITE)

    def update(self) -> py_trees.common.Status:
        self.get_blackboard_client().set(self.variable_name, self.variable_value)
        return Status.SUCCESS
