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

from scenario_execution.model.types import ParameterDeclaration, StructuredDeclaration, VariableDeclaration, ScenarioDeclaration, Declaration
from scenario_execution.model.model_base_visitor import ModelBaseVisitor
from scenario_execution.model.error import OSC2ParsingError


def create_py_tree_blackboard(model, tree, logger, log_tree):
    model_blackboard = ModelToBlackboard(logger)
    try:
        model_blackboard.build(model, tree, log_tree)
    except OSC2ParsingError as e:
        raise ValueError(f'Error while creating py-tree: {e}') from e


class ModelToBlackboard(object):

    def __init__(self, logger):
        self.logger = logger

    def build(self, model, tree, log_tree):

        self.blackboard = tree.attach_blackboard_client(name="ModelToBlackboard")
        behavior_builder = self.BehaviorInit(self.logger, tree)
        behavior_builder.visit(model)

    class BehaviorInit(ModelBaseVisitor):
        def __init__(self, logger, tree) -> None:
            super().__init__()
            self.logger = logger
            self.blackboard = tree.attach_blackboard_client(name="ModelToPyTree")

        def visit_parameter_declaration(self, node: ParameterDeclaration):
            parameter_type = node.get_type()[0]

            if isinstance(parameter_type, StructuredDeclaration) and self.needs_blackboard_entry(node):
                self.create_blackboard_entries(parameter_type, node.get_qualified_name())

        def create_blackboard_entries(self, elem, prefix):
            for variable_dec in elem.find_children_of_type(VariableDeclaration):
                fqn = prefix + "/" + variable_dec.name
                self.blackboard.register_key(fqn, access=py_trees.common.Access.WRITE)
                setattr(self.blackboard, fqn, variable_dec.get_resolved_value())

            for child in elem.find_children_of_type(ParameterDeclaration):
                child_type = child.get_type()[0]
                if isinstance(child_type, Declaration):
                    self.create_blackboard_entries(child_type, prefix + "/" + child.name)

        def needs_blackboard_entry(self, node):
            current = node.get_parent()
            while current:
                if isinstance(current, ScenarioDeclaration):
                    return True
                current = current.get_parent()
            return False

        def visit_variable_declaration(self, node: VariableDeclaration):
            if self.needs_blackboard_entry(node):
                blackboard_var_name = node.get_fully_qualified_var_name()
                self.blackboard.register_key(blackboard_var_name, access=py_trees.common.Access.WRITE)
                setattr(self.blackboard, blackboard_var_name, node.get_resolved_value())
