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


from .types import ModelElement, CompilationUnit, PhysicalTypeDeclaration, UnitDeclaration, SIBaseExponent, EnumDeclaration, EnumMemberDeclaration, EnumValueReference, InheritsCondition, StructDeclaration, StructInherits, ActionDeclaration, ActionInherits, ActorDeclaration, ActorInherits, FieldAccessExpression, FallExpression, FloatLiteral, FunctionApplicationExpression, Argument, BehaviorInvocation, BinaryExpression, BoolLiteral, CallDirective, CastExpression, CoverDeclaration, DoDirective, ElapsedExpression, ElementAccessExpression, DoMember, EmitDirective, EnumTypeExtension, EventCondition, EventDeclaration, EventFieldDecl, EventReference, EveryExpression, GlobalParameterDeclaration, Identifier, IdentifierReference, IntegerLiteral, KeepConstraintDeclaration, ListExpression, LogicalExpression, MethodBody, MethodDeclaration, ModifierDeclaration, ModifierInvocation, NamedArgument, OnDirective, ParameterDeclaration, PhysicalLiteral, ParameterReference, PositionalArgument, RangeExpression, RelationExpression, RecordDeclaration, RemoveDefaultDeclaration, RiseExpression, SampleExpression, ScenarioInherits, SIUnitSpecifier, StringLiteral, ScenarioDeclaration, StructuredTypeExtension, TernaryExpression, TypeTestExpression, UnaryExpression, Type, UntilDirective, VariableDeclaration, WaitDirective


class BaseVisitor(object):
    def visit(self, tree):
        return tree.accept(self)

    def visit_children(self, node):
        result = self.default_result()
        n = node.get_child_count()
        for i in range(n):
            if not self.should_visit_next_child(node, result):
                return result

            c = node.get_child(i)
            if isinstance(c, ModelElement):
                child_result = c.accept(self)
                result = self.aggregate_result(result, child_result)

        return result

    def default_result(self):
        return []

    def aggregate_result(self, aggregate, next_result):
        if aggregate:
            return [aggregate, next_result]
        else:
            return next_result

    def should_visit_next_child(self, node, current_result):
        return True


class ModelBaseVisitor(BaseVisitor):  # pylint: disable=too-many-public-methods
    def visit_compilation_unit(self, node: CompilationUnit):
        return self.visit_children(node)

    def visit_physical_type_declaration(self, node: PhysicalTypeDeclaration):
        return self.visit_children(node)

    def visit_unit_declaration(self, node: UnitDeclaration):
        return self.visit_children(node)

    def visit_si_base_exponent(self, node: SIBaseExponent):
        return self.visit_children(node)

    def visit_enum_declaration(self, node: EnumDeclaration):
        return self.visit_children(node)

    def visit_enum_member_declaration(self, node: EnumMemberDeclaration):
        return self.visit_children(node)

    def visit_enum_value_reference(self, node: EnumValueReference):
        return self.visit_children(node)

    def visit_inherits_condition(self, node: InheritsCondition):
        return self.visit_children(node)

    def visit_struct_declaration(self, node: StructDeclaration):
        return self.visit_children(node)

    def visit_struct_inherits(self, node: StructInherits):
        return self.visit_children(node)

    def visit_actor_declaration(self, node: ActorDeclaration):
        return self.visit_children(node)

    def visit_actor_inherits(self, node: ActorInherits):
        return self.visit_children(node)

    def visit_scenario_declaration(self, node: ScenarioDeclaration):
        return self.visit_children(node)

    def visit_scenario_inherits(self, node: ScenarioInherits):
        return self.visit_children(node)

    def visit_action_declaration(self, node: ActionDeclaration):
        return self.visit_children(node)

    def visit_action_inherits(self, node: ActionInherits):
        return self.visit_children(node)

    def visit_modifier_declaration(self, node: ModifierDeclaration):
        return self.visit_children(node)

    def visit_enum_type_extension(self, node: EnumTypeExtension):
        return self.visit_children(node)

    def visit_structured_type_extension(self, node: StructuredTypeExtension):
        return self.visit_children(node)

    def visit_global_parameter_declaration(
        self, node: GlobalParameterDeclaration
    ):
        return self.visit_children(node)

    def visit_parameter_declaration(self, node: ParameterDeclaration):
        return self.visit_children(node)

    def visit_parameter_reference(self, node: ParameterReference):
        return self.visit_children(node)

    def visit_variable_declaration(self, node: VariableDeclaration):
        return self.visit_children(node)

    def visit_event_declaration(self, node: EventDeclaration):
        return self.visit_children(node)

    def visit_event_reference(self, node: EventReference):
        return self.visit_children(node)

    def visit_event_field_declaration(self, node: EventFieldDecl):
        return self.visit_children(node)

    def visit_event_condition(self, node: EventCondition):
        return self.visit_children(node)

    def visit_method_declaration(self, node: MethodDeclaration):
        return self.visit_children(node)

    def visit_method_body(self, node: MethodBody):
        return self.visit_children(node)

    def visit_cover_declaration(self, node: CoverDeclaration):
        return self.visit_children(node)

    def visit_record_declaration(self, node: RecordDeclaration):
        return self.visit_children(node)

    def visit_argument(self, node: Argument):
        return self.visit_children(node)

    def visit_named_argument(self, node: NamedArgument):
        return self.visit_children(node)

    def visit_positional_argument(self, node: PositionalArgument):
        return self.visit_children(node)

    def visit_keep_constraint_declaration(self, node: KeepConstraintDeclaration):
        return self.visit_children(node)

    def visit_remove_default_declaration(self, node: RemoveDefaultDeclaration):
        return self.visit_children(node)

    def visit_on_directive(self, node: OnDirective):
        return self.visit_children(node)

    def visit_do_directive(self, node: DoDirective):
        return self.visit_children(node)

    def visit_do_member(self, node: DoMember):
        return self.visit_children(node)

    def visit_wait_directive(self, node: WaitDirective):
        return self.visit_children(node)

    def visit_emit_directive(self, node: EmitDirective):
        return self.visit_children(node)

    def visit_call_directive(self, node: CallDirective):
        return self.visit_children(node)

    def visit_until_directive(self, node: UntilDirective):
        return self.visit_children(node)

    def visit_behavior_invocation(self, node: BehaviorInvocation):
        return self.visit_children(node)

    def visit_modifier_invocation(self, node: ModifierInvocation):
        return self.visit_children(node)

    def visit_rise_expression(self, node: RiseExpression):
        return self.visit_children(node)

    def visit_fall_expression(self, node: FallExpression):
        return self.visit_children(node)

    def visit_elapsed_expression(self, node: ElapsedExpression):
        return self.visit_children(node)

    def visit_every_expression(self, node: EveryExpression):
        return self.visit_children(node)

    def visit_sample_expression(self, node: SampleExpression):
        return self.visit_children(node)

    def visit_cast_expression(self, node: CastExpression):
        return self.visit_children(node)

    def visit_type_test_expression(self, node: TypeTestExpression):
        return self.visit_children(node)

    def visit_element_access_expression(self, node: ElementAccessExpression):
        return self.visit_children(node)

    def visit_function_application_expression(self, node: FunctionApplicationExpression):
        return self.visit_children(node)

    def visit_binary_expression(self, node: BinaryExpression):
        return self.visit_children(node)

    def visit_unary_expression(self, node: UnaryExpression):
        return self.visit_children(node)

    def visit_ternary_expression(self, node: TernaryExpression):
        return self.visit_children(node)

    def visit_list_expression(self, node: ListExpression):
        return self.visit_children(node)

    def visit_range_expression(self, node: RangeExpression):
        return self.visit_children(node)

    def visit_physical_literal(self, node: PhysicalLiteral):
        return self.visit_children(node)

    def visit_integer_literal(self, node: IntegerLiteral):
        return self.visit_children(node)

    def visit_float_literal(self, node: FloatLiteral):
        return self.visit_children(node)

    def visit_bool_literal(self, node: BoolLiteral):
        return self.visit_children(node)

    def visit_string_literal(self, node: StringLiteral):
        return self.visit_children(node)

    def visit_type(self, node: Type):
        return self.visit_children(node)

    def visit_identifier(self, node: Identifier):
        return self.visit_children(node)

    def visit_identifier_reference(self, node: IdentifierReference):
        return self.visit_children(node)

    def visit_field_access_expression(self, node: FieldAccessExpression):
        return self.visit_children(node)

    def visit_logical_expression(self, node: LogicalExpression):
        return self.visit_children(node)

    def visit_si_unit_specifier(self, node: SIUnitSpecifier):
        return self.visit_children(node)

    def visit_relation_expression(self, node: RelationExpression):
        return self.visit_children(node)
