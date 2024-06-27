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

import os
import py_trees
from scenario_execution.actions.base_action import BaseAction

class TestActorSetValue(BaseAction):

    def __init__(self, name):
        super().__init__(name)
        #self.file_name = file_name
        self.i = 0
        self.associated_actor = None
        self.value = None
    
    def setup(self, **kwargs):
        return super().setup(**kwargs)

    def execute(self, associated_actor, value):
        self.associated_actor = associated_actor
        self.value = value
        
    def update(self) -> py_trees.common.Status:
    #     if os.path.isfile(self.file_name):
    #         self.feedback_message = f"File '{self.file_name}' exists"  # pylint: disable= attribute-defined-outside-init
        print(self.associated_actor)
        self.i += 1
        if self.i < 10:
            return py_trees.common.Status.RUNNING
        else:
            return py_trees.common.Status.SUCCESS
    #     else:
    #         self.feedback_message = f"File '{self.file_name}' does not exist"  # pylint: disable= attribute-defined-outside-init
    #         return py_trees.common.Status.FAILURE
