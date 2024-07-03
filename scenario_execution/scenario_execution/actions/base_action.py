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
from scenario_execution.model.types import ParameterDeclaration, ScenarioDeclaration


class BaseAction(py_trees.behaviour.Behaviour):

    # subclasses might implement __init__() with the same arguments as defined in osc
    # CAUTION: __init__() only gets the initial parameter values. In case variables get modified during scenario execution,
    #          the latest values are available in execute() only.
    def __init__(self):
        self.blackboard = None
        super().__init__(self.__class__.__name__)

    # Subclasses might implement execute() with the same arguments as defined in osc.
    # def execute(self):

    # Subclasses might override shutdown() in order to cleanup on scenario shutdown.
    def shutdown(self):
        pass

    #############
    # Internal methods below, do not override in subclass
    #############

    def initialise(self):
        execute_method = getattr(self, "execute", None)
        if execute_method is not None and callable(execute_method):
            final_args = self.model.get_resolved_value(self.get_blackboard_client())

            if self.model.actor:
                final_args["associated_actor"] = self.model.actor.get_resolved_value()
                final_args["associated_actor"]["name"] = self.model.actor.name
            self.execute(**final_args)

    def _set_name_and_model(self, name, model):
        self.name = name
        self.model = model

    def get_blackboard_client(self):
        if self.blackboard:
            return self.blackboard

        def get_blackboard_namespace(node: ParameterDeclaration):
            parent = node.get_parent()
            while parent is not None and not isinstance(parent, ScenarioDeclaration):
                parent = parent.get_parent()
            if parent:
                return parent.name
            else:
                return None

        self.blackboard = self.attach_blackboard_client(name=self.name, namespace=get_blackboard_namespace(self.model))
        return self.blackboard
