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

import operator as op

class Compare(BaseAction):
    """
    Class to evaluate an expression
    """

    def __init__(self):
        super().__init__(resolve_variable_reference_arguments_in_execute=False)
        self.target_variable = None
        self.left_value = None
        self.right_value = None

    def execute(self, left_value: object, operator: str, right_value: object):
        self.operator = self.get_operator(operator)
        self.left_value = left_value
        self.right_value = right_value

    def update(self) -> py_trees.common.Status:
        if isinstance(self.left_value, VariableReference):
            left_value = self.left_value.get_value()
        else:
            left_value = self.left_value
        
        if isinstance(self.right_value, VariableReference):
            right_value = self.right_value.get_value()
        else:
            right_value = self.right_value

        if self.operator(left_value, right_value):
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE

    def get_operator(self, operator_string: str):
        operator = None
        if operator_string == "==":
            operator = op.eq
        elif operator_string == "!=":
            operator = op.ne
        elif operator_string == "<":
            operator = op.lt
        elif operator_string == "<=":
            operator = op.le
        elif operator_string == ">":
            operator = op.gt
        elif operator_string == ">=":
            operator = op.ge
        else:
            raise ActionError(f"Unknown expression operator {operator_string}.", action=self)
        return operator