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

from scenario_execution.model.types import ActionDeclaration, ActionInherits, EnumDeclaration, EnumValueReference, KeepConstraintDeclaration, EmitDirective, Type

from .types import Argument, UnitDeclaration, EnumValueReference, StructInherits, ActionDeclaration, ActionInherits, ActorInherits, FieldAccessExpression,  BehaviorInvocation, EmitDirective, GlobalParameterDeclaration, IdentifierReference, Parameter, MethodBody, MethodDeclaration, ModelElement, StructuredDeclaration, KeepConstraintDeclaration, NamedArgument, ParameterDeclaration, PhysicalLiteral,  PositionalArgument,  RelationExpression, ScenarioInherits, SIUnitSpecifier,  Type, EnumMemberDeclaration, ListExpression, print_tree, ScenarioDeclaration, VariableDeclaration

from .model_base_visitor import ModelBaseVisitor
from scenario_execution.model.error import OSC2ParsingError
import importlib
import inspect
import py_trees

def resolve_internal_model(model, tree, logger, log_tree):
    osc2scenario_resolver = ModelResolver(logger, tree)
    try:
        osc2scenario_resolver.visit(model)
    except OSC2ParsingError as e:
        raise ValueError(
            f'Error while creating tree:\nTraceback <line: {e.line}, column: {e.column}> in "{e.filename}":\n  -> {e.context}\n{e.__class__.__name__}: {e.msg}') from e

    if log_tree:
        logger.info("----Internal model (resolved)-----")
        print_tree(model, logger)


class ModelResolver(ModelBaseVisitor):

    def __init__(self, logger, tree) -> None:
        super().__init__()
        self.logger = logger
        self.blackboard = tree.attach_blackboard_client(name="ModelResolver")

    def visit_physical_literal(self, node: PhysicalLiteral):
        unit = node.resolve(node.unit)
        if unit is None:
            raise OSC2ParsingError(
                msg=f'Physical unit "{node.unit}" not defined.', context=node.get_ctx())
        node.unit = unit
        si_unit_specifier = node.unit.find_first_child_of_type(SIUnitSpecifier)
        if si_unit_specifier is None:
            raise OSC2ParsingError(
                msg=f'SIUnitSpecifier of physical unit "{node.unit.name}" not defined.', context=node.get_ctx())

    def visit_identifier_reference(self, node: IdentifierReference):
        if '.' in node.ref:  # first level of members can also be referenced (e.g. methods)
            splitted = node.ref.split('.')
            if len(splitted) != 2:
                raise OSC2ParsingError(
                    msg=f'Identifier "{node.ref}" with more than one sub-level not supported.', context=node.get_ctx())
                
            member = splitted[1]
            resolved = node.resolve(splitted[0])
            if resolved:
                if isinstance(resolved, ParameterDeclaration):
                    print(member)
                    typ = resolved.get_type()[0]
                    print(typ)
                    resolved = typ.get_named_child(member)
                    print("resolved")
                    # for child in resolved.__children:
                    #     if member == child.name:
                    #         return child
                else:
                    resolved = resolved.get_named_child(member)
        else:
            resolved = node.resolve(node.ref)
        if resolved is None:
            raise OSC2ParsingError(
                msg=f'Identifier "{node.ref}" not defined.', context=node.get_ctx())
        node.ref = resolved

    def visit_type(self, node: Type):
        type_string = node.type_def
        if node.is_list:
            if not type_string.startswith('listof'):
                raise OSC2ParsingError(
                    msg=f'Invalid list type "{type_string}".', context=node.get_ctx())
            type_string = type_string.removeprefix('listof')

        if type_string not in ['string', 'int', 'bool', 'float', 'uint']:
            resolved = node.resolve(type_string)
            if resolved is None:
                raise OSC2ParsingError(
                    msg=f'Type "{type_string}" not defined.', context=node.get_ctx())
            node.type_def = resolved

    def visit_unit_declaration(self, node: UnitDeclaration):
        resolved = node.resolve(node.physical_name)
        if resolved is None:
            raise OSC2ParsingError(
                msg=f'Unit declaration refers to unknown physical name "{node.physical_name}".', context=node.get_ctx())
        node.physical_name = resolved

    def visit_keep_constraint_declaration(self, node: KeepConstraintDeclaration):
        self.visit_children(node)
        if node.get_child_count() == 1 and isinstance(node.get_child(0), RelationExpression) and node.get_child(0).get_child_count() == 2:
            if node.get_child(0).operator != "==":
                raise OSC2ParsingError(
                    msg=f'Only relation "==" is currently supported in "keep".', context=node.get_ctx())
            field_exp = node.get_child(0).find_first_child_of_type(FieldAccessExpression)

            if not field_exp:
                raise OSC2ParsingError(
                    msg=f'FieldAccessExpression not found.', context=node.get_ctx())
            if not field_exp.field_name.startswith('it.'):
                raise OSC2ParsingError(
                    msg=f'FieldAccessExpression only supports "it." prefix, not "{field_exp.field_name}".', context=node.get_ctx())

            definition, _ = node.get_parent().get_type()
            if not isinstance(definition, StructuredDeclaration):
                raise OSC2ParsingError(
                    msg=f'keep expected reference to structured type.', context=node.get_ctx())

            parameters = definition.get_resolved_value()
            expected_member_name = field_exp.field_name.removeprefix("it.")

            member_path = expected_member_name.split('.')
            current_params = parameters
            for current in member_path:
                if current not in current_params:
                    raise OSC2ParsingError(
                        msg=f'keep reference {field_exp.field_name} not found in {definition.name}. Unknown key "{current}"', context=node.get_ctx())
                current_params = current_params[current]
        else:
            raise OSC2ParsingError(
                msg=f'Keep uses unsupported expression: allowed "==" only.', context=node.get_ctx())

    def check_parameter_type(self, node: Parameter):
        val = node.get_value_child()
        if isinstance(val, KeepConstraintDeclaration):
            pass
        else:
            if val is not None:
                val_type = val.get_type_string()
                param_type = node.field_type
                # if isinstance(node.field_type, Type) and node.field_type.is_list:
                #     param_type = "listof" + param_type
                if param_type == 'uint':
                    param_type = 'int'
                if val_type != param_type:
                    raise OSC2ParsingError(
                        msg=f'Parameter type "{param_type}" does not match value type "{val_type}".', context=node.get_ctx())

                # check list entries
                if isinstance(val, ListExpression):
                    expected_type = param_type.removeprefix('listof')
                    for child in val.get_children():
                        member_type = child.get_type_string()
                        if expected_type != member_type:
                            raise OSC2ParsingError(
                                msg=f'List entry does not have valid type. Expected "{expected_type}", found "{member_type}".', context=node.get_ctx())

    def visit_global_parameter_declaration(self, node: GlobalParameterDeclaration):
        self.visit_children(node)
        self.check_parameter_type(node)

    def visit_parameter_declaration(self, node: ParameterDeclaration):
        self.visit_children(node)
        self.check_parameter_type(node)
        if isinstance(node.get_parent(), ScenarioDeclaration):
            type_def = node.get_type()[0]
            if isinstance(type_def, StructuredDeclaration):
                for child in type_def.find_children_of_type(VariableDeclaration):
                    key = node.get_parent().name + '/' + node.name + '/' + child.name
                    self.blackboard.register_key(key=key, access=py_trees.common.Access.WRITE)
                    setattr(self.blackboard, key, node.get_type()[0])
            

    def visit_actor_inherits(self, node: ActorInherits):
        resolved = node.resolve(node.actor)
        if resolved is None:
            raise OSC2ParsingError(
                msg=f'Actor inherits from unknown "{node.actor}".', context=node.get_ctx())
        node.actor = resolved

    def visit_struct_inherits(self, node: StructInherits):
        resolved = node.resolve(node.struct_name)
        if resolved is None:
            raise OSC2ParsingError(
                msg=f'Struct inherits from unknown "{node.struct_name}".', context=node.get_ctx())
        node.struct_name = resolved

    def visit_action_inherits(self, node: ActionInherits):
        resolved = node.resolve(node.qualified_behavior_name)
        if resolved is None:
            raise OSC2ParsingError(
                msg=f'Action inherits from unknown "{node.qualified_behavior_name}".', context=node.get_ctx())
        node.qualified_behavior_name = resolved

    def visit_scenario_inherits(self, node: ScenarioInherits):
        resolved = node.resolve(node.qualified_behavior_name)
        if resolved is None:
            raise OSC2ParsingError(
                msg=f'Scenario inherits from unknown "{node.qualified_behavior_name}".', context=node.get_ctx())
        node.qualified_behavior_name = resolved

    def visit_behavior_invocation(self, node: BehaviorInvocation):
        qualified_behavior_name = node.behavior
        if node.actor:
            resolved_actor = node.resolve(node.actor)
            if resolved_actor is None:
                raise OSC2ParsingError(
                    msg=f'BehaviorInvocation refers to unknown actor "{node.actor}".', context=node.get_ctx())
            node.actor = resolved_actor

            current, _ = node.actor.get_type()
            resolved = None
            while current and resolved is None:  # look for action in all base_types
                qualified_behavior_name = current.name + "." + node.behavior
                resolved = node.resolve(qualified_behavior_name)
                current = current.get_base_type()
        else:
            resolved = node.resolve(node.behavior)

        if not resolved:
            raise OSC2ParsingError(
                msg=f'BehaviorInvocation uses unknown behavior "{qualified_behavior_name}".', context=node.get_ctx())
        node.behavior = resolved

        pos_arg_count = 0
        named = False
        param_keys = node.get_parameter_names()
        max_named_arg_count = len(param_keys)
        for child in node.get_children():
            if isinstance(child, NamedArgument):
                named = True
                if child.name not in param_keys:
                    raise OSC2ParsingError(
                        msg=f'Named argument {child.name} unknown.', context=node.get_ctx())

            elif isinstance(child, PositionalArgument):
                if named:
                    raise OSC2ParsingError(
                        msg=f'No positional argument allowed after named.', context=node.get_ctx())
                if pos_arg_count >= max_named_arg_count:
                    raise OSC2ParsingError(
                        msg=f'Too many positional arguments.', context=node.get_ctx())
                pos_arg_count += 1

        super().visit_behavior_invocation(node)

    def visit_action_declaration(self, node: ActionDeclaration):
        super().visit_action_declaration(node)

        param_names = node.get_parameter_names()
        if 'name' in param_names:
            raise OSC2ParsingError(
                msg=f'ActionDeclaration {node.name} uses reserved paramater name "name".', context=node.get_ctx())
        if 'associated_actor' in param_names:
            raise OSC2ParsingError(
                msg=f'ActionDeclaration {node.name} uses reserved paramater name "associated_actor".', context=node.get_ctx())

        actor_name = None
        if "." in node.qualified_behavior_name:
            actor_name = node.qualified_behavior_name.split('.')[0]
        if actor_name:
            resolved_actor = node.resolve(actor_name)
            if resolved_actor is None:
                raise OSC2ParsingError(
                    msg=f'ActionDeclaration {node.name} refers to unknown actor "{actor_name}".', context=node.get_ctx())
            node.actor = resolved_actor

    def visit_enum_value_reference(self, node: EnumValueReference):
        # skip parameter level (to allow parameter names to be similar to enum-type-names)
        enum_type = node.get_parent().resolve(node.enum_name)
        if enum_type is None:
            raise OSC2ParsingError(
                msg=f'Enum type {node.enum_name} unknown.', context=node.get_ctx())
        node.enum_name = enum_type

        member = None
        for child in enum_type.get_children():
            if isinstance(child, ModelElement) and child.member_name == node.enum_member_name:
                member = child
                break
        if member is None:
            raise OSC2ParsingError(
                msg=f'Enum type {node.enum_name} does not have a member "{node.enum_member_name}".', context=node.get_ctx())
        node.enum_member_name = member

    def visit_emit_directive(self, node: EmitDirective):
        if node.event_name not in ['start', 'end', 'fail']:
            node.event = node.resolve(node.event_name)
            if node.event is None:
                raise OSC2ParsingError(
                    msg=f'EmitDirective refers to unknown event {node.event_name}.', context=node.get_ctx())

    def visit_enum_declaration(self, node: EnumDeclaration):
        next_numeric_val = 0
        for child in node.get_children():
            if isinstance(child, EnumMemberDeclaration):
                if child.numeric_value is None:
                    child.numeric_value = next_numeric_val
                    next_numeric_val += 1
                else:
                    next_numeric_val = child.numeric_value + 1

        return super().visit_enum_declaration(node)

    def visit_method_declaration(self, node: MethodDeclaration):
        super().visit_method_declaration(node)
        body = node.find_first_child_of_type(MethodBody)
        if body.type_ref == 'external':
            if not body.external_name:
                raise OSC2ParsingError(msg=f'No external name defined.', context=node.get_ctx())
            package, method = body.external_name.rsplit('.', 1)
            mod = importlib.import_module(package)
            body.external_name = getattr(mod, method)

            external_args = inspect.getfullargspec(body.external_name).args

            args = node.find_children_of_type(Argument)
            for arg in args:
                param_type, _ = arg.get_type()
                if isinstance(param_type, ModelElement):
                    param_type = param_type.get_base_type()
                if arg.name not in external_args:
                    raise OSC2ParsingError(msg=f'Argument "{arg.name}" not found in external method definition', context=node.get_ctx())
                external_args.remove(arg.name)
        else:
            raise OSC2ParsingError(
                msg=f'MethodDeclaration currently only supports "external", not "{body.type_ref}"', context=node.get_ctx())
