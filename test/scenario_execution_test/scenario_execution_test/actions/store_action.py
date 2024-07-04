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


class StoreAction(BaseAction):

    def setup(self, **kwargs):
        self.i = 0
        self.value = None
        self.file_path = None
        return super().setup(**kwargs)

    def execute(self, file_path, value):
        self.file_path = file_path
        self.value = value

    def update(self) -> py_trees.common.Status:
        with open(self.file_path, 'w') as f:
            f.write(str(self.value))
        return py_trees.common.Status.SUCCESS
