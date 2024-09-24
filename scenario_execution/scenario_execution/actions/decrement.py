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

import py_trees  # pylint: disable=import-error
from scenario_execution.actions.base_action import BaseAction, ActionError
from scenario_execution.model.types import VariableReference


class Decrement(BaseAction):
    """
    Class to decrement the value of a variable
    """

    def __init__(self):
        super().__init__(resolve_variable_reference_arguments_in_execute=False)
        self.target_variable = None

    def execute(self, target_variable: object):
        if not isinstance(target_variable, VariableReference):
            raise ActionError(
                f"'target_variable' is expected to be a variable reference but is {type(target_variable).__name__}.", action=self)
        self.target_variable = target_variable

    def update(self) -> py_trees.common.Status:
        self.target_variable.set_value(self.target_variable.get_value() - 1)
        return py_trees.common.Status.SUCCESS
