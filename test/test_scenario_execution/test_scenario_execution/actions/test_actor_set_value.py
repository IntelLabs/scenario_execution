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
from scenario_execution.model.types import ModelElement, ParameterDeclaration, ScenarioDeclaration

class TestActorSetValue(BaseAction):

    def __init__(self, name):
        super().__init__(name)
        self.i = 0
        self.associated_actor = None
        self.value = None
        
    def get_blackboard_client(self):
        def get_blackboard_namespace(node: ParameterDeclaration):
            parent = node.get_parent()
            while parent is not None and not isinstance(parent, ScenarioDeclaration):
                parent = parent.get_parent()
            if parent:
                return parent.name
            else:
                return None
        
        blackboard_client = self.attach_blackboard_client(self.name, get_blackboard_namespace(self.model))
        return blackboard_client
        
    def set_variable(self, model_instance, variable_name, value):
        blackboard = self.get_blackboard_client()
    
        def get_fully_qualified_model_name(node: ModelElement):
            name = node.name
            parent = node.get_parent()
            while not isinstance(parent, ScenarioDeclaration):
                name = parent.name + "_" + name
                parent = parent.get_parent()
            return name
        model_blackboard_name = get_fully_qualified_model_name(model_instance)
        model_blackboard_name += "_" + variable_name
        blackboard.register_key(model_blackboard_name, access=py_trees.common.Access.WRITE)
        current = getattr(blackboard, model_blackboard_name)
        self.logger.debug(f"Set variable '{model_blackboard_name}' from '{current}' to '{value}'")
        setattr(blackboard, model_blackboard_name, value)

    def setup(self, **kwargs):
        return super().setup(**kwargs)

    def execute(self, associated_actor, value):
        self.associated_actor = associated_actor
        self.value = value
        
    def update(self) -> py_trees.common.Status:
    #     if os.path.isfile(self.file_name):
    #         self.feedback_message = f"File '{self.file_name}' exists"  # pylint: disable= attribute-defined-outside-init
        # print(self.associated_actor)
        self.i += 1
        if self.i < 3:
            return py_trees.common.Status.RUNNING
        else:
            self.set_variable(self.model.actor, "test", self.value)
            return py_trees.common.Status.SUCCESS
    #     else:
    #         self.feedback_message = f"File '{self.file_name}' does not exist"  # pylint: disable= attribute-defined-outside-init
    #         return py_trees.common.Status.FAILURE
