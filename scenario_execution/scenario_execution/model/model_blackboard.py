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

from scenario_execution.model.types import ParameterDeclaration, StructuredDeclaration, VariableDeclaration
from scenario_execution.model.model_base_visitor import ModelBaseVisitor
from scenario_execution.model.error import OSC2ParsingError


def create_py_tree_blackboard(model, tree, logger, log_tree):
    model_blackboard = ModelToBlackboard(logger)
    try:
        model_blackboard.build(model, tree, log_tree)
    except OSC2ParsingError as e:
        raise ValueError(
            f'Error while creating py-tree:\nTraceback <line: {e.line}, column: {e.column}> in "{e.filename}":\n  -> {e.context}\n{e.__class__.__name__}: {e.msg}') from e


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
            super().visit_parameter_declaration(node)
            parameter_type = node.get_type()[0]
            if isinstance(parameter_type, StructuredDeclaration):
                for variable_dec in parameter_type.find_children_of_type(VariableDeclaration):
                    prefix = node.get_fully_qualified_var_name(include_scenario=True)
                    blackboard_var_name = prefix + "/" + variable_dec.name

                    self.blackboard.register_key(blackboard_var_name, access=py_trees.common.Access.WRITE)
                    setattr(self.blackboard, blackboard_var_name, variable_dec.get_resolved_value())
