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
from scenario_execution.actions.base_action import BaseAction


class ActorSetValue(BaseAction):

    def setup(self, **kwargs):
        self.i = 0
        self.value = None
        return super().setup(**kwargs)

    def execute(self, associated_actor, value):
        self.associated_actor = associated_actor
        self.value = value

    def update(self) -> py_trees.common.Status:
        self.i += 1
        if self.i < 3:
            return py_trees.common.Status.RUNNING
        else:
            self.set_variable(self.model.actor, "test", self.value)
            return py_trees.common.Status.SUCCESS
