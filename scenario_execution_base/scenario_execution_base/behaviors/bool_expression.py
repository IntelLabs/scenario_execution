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
Behavior for bool expression in event_condition
"""
import py_trees
from py_trees.common import Status


class BoolExpression(py_trees.behaviour.Behaviour):
    """
    Class for bool expression in event_condition

    Args:
        value [bool]: boolean value to check
    """

    def __init__(self, value: bool):
        super().__init__(name=self.__class__.__name__)
        self.value = value

    def update(self) -> Status:
        """
        Return success if the bool value is true.
        """
        if self.value:
            return Status.SUCCESS
        else:
            return Status.FAILURE
