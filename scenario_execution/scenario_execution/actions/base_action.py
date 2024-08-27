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
from scenario_execution.model.error import OSC2Error


class BaseAction(py_trees.behaviour.Behaviour):

    # subclasses might implement __init__() with the same arguments as defined in osc
    # CAUTION: __init__() only gets the initial parameter values. In case variables get modified during scenario execution,
    #          the latest values are available in execute() only.
    def __init__(self, resolve_variable_reference_arguments_in_execute=True):
        self._model = None
        self.logger = None
        self.blackboard = None
        self.resolve_variable_reference_arguments_in_execute = resolve_variable_reference_arguments_in_execute
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

            if self.resolve_variable_reference_arguments_in_execute:
                final_args = self._model.get_resolved_value(self.get_blackboard_client())
            else:
                final_args = self._model.get_resolved_value_with_variable_references(self.get_blackboard_client())

            if self._model.actor:
                final_args["associated_actor"] = self._model.actor.get_resolved_value(self.get_blackboard_client())
                final_args["associated_actor"]["name"] = self._model.actor.name
            self.execute(**final_args)  # pylint: disable=no-member

    def _set_base_properities(self, name, model, logger):
        self.name = name
        self._model = model
        self.logger = logger

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

        self.blackboard = self.attach_blackboard_client(name=self.name, namespace=get_blackboard_namespace(self._model))
        return self.blackboard

    def register_access_to_associated_actor_variable(self, variable_name):
        if not self._model.actor:
            raise ValueError("Model does not have 'actor'.")
        blackboard = self.get_blackboard_client()
        model_blackboard_name = self._model.actor.get_fully_qualified_var_name(include_scenario=False)
        model_blackboard_name += "/" + variable_name
        blackboard.register_key(model_blackboard_name, access=py_trees.common.Access.WRITE)
        return model_blackboard_name

    def set_associated_actor_variable(self, variable_name, value):
        model_blackboard_name = self.register_access_to_associated_actor_variable(variable_name)
        self.logger.debug(f"Set variable '{model_blackboard_name}' to '{value}'")
        setattr(self.get_blackboard_client(), model_blackboard_name, value)

    def get_associated_actor_variable(self, variable_name):
        model_blackboard_name = self.register_access_to_associated_actor_variable(variable_name)
        self.logger.debug(f"Get variable '{model_blackboard_name}'")
        return getattr(self.get_blackboard_client(), model_blackboard_name)


class ActionError(OSC2Error):

    def __init__(self, msg: str, action: BaseAction, *args) -> None:
        if action is not None:
            ctx = action._model.get_ctx()
        super().__init__(msg, ctx, *args)
