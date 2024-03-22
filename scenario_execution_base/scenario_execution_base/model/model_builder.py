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


from .types import CompilationUnit, PhysicalTypeDeclaration, UnitDeclaration, EnumDeclaration, EnumMemberDeclaration, EnumValueReference, StructDeclaration, StructInherits, ActionDeclaration, ActionInherits, ActorDeclaration, ActorInherits, FieldAccessExpression,  FloatLiteral, FunctionApplicationExpression, Argument, BehaviorInvocation, BinaryExpression, BoolLiteral, DoDirective, ElapsedExpression, DoMember, EmitDirective,  EventCondition, EventDeclaration, EventFieldDecl, EventReference,  GlobalParameterDeclaration, Identifier, IdentifierReference, IntegerLiteral, KeepConstraintDeclaration,  LogicalExpression, MethodBody,    NamedArgument, OnDirective, ParameterDeclaration, PhysicalLiteral, ParameterReference, PositionalArgument,  RelationExpression,     ScenarioInherits, SIUnitSpecifier, StringLiteral, ScenarioDeclaration, StructuredTypeExtension,  Type, VariableDeclaration, WaitDirective, ListExpression


from ..osc2_parsing.OpenSCENARIO2Parser import OpenSCENARIO2Parser
from ..osc2_parsing.OpenSCENARIO2Listener import OpenSCENARIO2Listener

from scenario_execution_base.model.error import OSC2ParsingError
import ast

from antlr4.tree.Tree import ParseTreeWalker
import os
from pkg_resources import iter_entry_points, resource_filename


class ModelBuilder(OpenSCENARIO2Listener):  # pylint: disable=too-many-public-methods

    def __init__(self, logger, parse_file_fct, file_name, log_model):
        super().__init__()
        self.__node_stack = []
        self.__cur_node = None
        self.__current_label = None
        self.model = None
        self.logger = logger
        self.imported_files = []
        self.parse_file = parse_file_fct
        self.current_file = file_name
        self.log_model = log_model

    def get_model(self):
        return self.model

    # Enter a parse tree produced by OpenSCENARIO2Parser#osc_file.
    def enterOsc_file(self, ctx: OpenSCENARIO2Parser.Osc_fileContext):
        if self.model is None:  # only on first file
            node = CompilationUnit()
            node.set_ctx(ctx, self.current_file)

            self.__node_stack.append(node)
            self.__cur_node = node
            self.model = node

    # Exit a parse tree produced by OpenSCENARIO2Parser#osc_file.
    def exitOsc_file(self, ctx: OpenSCENARIO2Parser.Osc_fileContext):
        pass

    # Enter a parse tree produced by OpenSCENARIO2Parser#preludeStatement.
    def enterPreludeStatement(self, ctx: OpenSCENARIO2Parser.PreludeStatementContext):
        pass

    # Exit a parse tree produced by OpenSCENARIO2Parser#preludeStatement.
    def exitPreludeStatement(self, ctx: OpenSCENARIO2Parser.PreludeStatementContext):
        pass

    # Enter a parse tree produced by OpenSCENARIO2Parser#importStatement.
    def enterImportStatement(self, ctx: OpenSCENARIO2Parser.ImportStatementContext):
        pass

    # Exit a parse tree produced by OpenSCENARIO2Parser#importStatement.
    def exitImportStatement(self, ctx: OpenSCENARIO2Parser.ImportStatementContext):
        pass

    # Enter a parse tree produced by OpenSCENARIO2Parser#importReference.
    def enterImportReference(self, ctx: OpenSCENARIO2Parser.ImportReferenceContext):
        file = None
        if ctx.StringLiteral():
            file = ctx.getText()
        if ctx.structuredIdentifier():
            import_reference_string = ""
            for child in ctx.structuredIdentifier().getChildren():
                import_reference_string += child.getText()
            import_reference = import_reference_string.split('.')
            self.logger.debug(f'import_reference is: {import_reference}')
            if len(import_reference) < 2 or import_reference[0] != 'osc':
                raise OSC2ParsingError(
                    msg=f'import_reference can only be of format osc.<library-name>, found "{import_reference}', context=ctx)

            library_name = ".".join(import_reference[1:])
            # iterate through all packages
            libraries_found = []
            for entry_point in iter_entry_points(group='scenario_execution.osc_libraries', name=None):
                if entry_point.name == library_name:
                    libraries_found.append(entry_point)
            if not libraries_found:
                raise OSC2ParsingError(
                    msg=f'No import library "{library_name}" found.', context=ctx)
            if len(libraries_found) > 1:
                pkgs = []
                for elem in libraries_found:
                    try:
                        lib_class = elem.load()
                        resource, _ = lib_class()
                        pkgs.append(resource)
                    except ModuleNotFoundError:
                        pkgs.append('<unknown>')
                        pass

                raise OSC2ParsingError(
                    msg=f'More than one import library for "{library_name}" found: {", ".join(pkgs)}', context=ctx)

            lib_class = libraries_found[0].load()
            resource, filename = lib_class()

            lib_osc_dir = resource_filename(resource, 'lib_osc')
            file = os.path.join(lib_osc_dir, filename)

        # Skip files that are already imported
        if file in self.imported_files:
            return

        imported_tree, errors = self.parse_file(file, self.log_model, f"{file} :")

        if errors:
            raise OSC2ParsingError(msg=f'{errors} parsing errors found in import {file}.', context=ctx)

        walker = ParseTreeWalker()
        prev_file = self.current_file
        self.current_file = file
        walker.walk(self, imported_tree)
        self.current_file = prev_file
        self.imported_files.append(import_reference)

    # Exit a parse tree produced by OpenSCENARIO2Parser#importReference.
    def exitImportReference(self, ctx: OpenSCENARIO2Parser.ImportReferenceContext):
        pass

    # Enter a parse tree produced by OpenSCENARIO2Parser#structuredIdentifier.
    def enterStructuredIdentifier(self, ctx: OpenSCENARIO2Parser.StructuredIdentifierContext):
        pass

    # Exit a parse tree produced by OpenSCENARIO2Parser#structuredIdentifier.
    def exitStructuredIdentifier(self, ctx: OpenSCENARIO2Parser.StructuredIdentifierContext):
        pass

    # Enter a parse tree produced by OpenSCENARIO2Parser#oscDeclaration.
    def enterOscDeclaration(self, ctx: OpenSCENARIO2Parser.OscDeclarationContext):
        pass

    # Exit a parse tree produced by OpenSCENARIO2Parser#oscDeclaration.
    def exitOscDeclaration(self, ctx: OpenSCENARIO2Parser.OscDeclarationContext):
        pass

    # Enter a parse tree produced by OpenSCENARIO2Parser#physicalTypeDeclaration.
    def enterPhysicalTypeDeclaration(self, ctx: OpenSCENARIO2Parser.PhysicalTypeDeclarationContext):
        self.__node_stack.append(self.__cur_node)
        type_name = ctx.physicalTypeName().getText()

        node = PhysicalTypeDeclaration(type_name)
        node.set_ctx(ctx, self.current_file)

        self.__cur_node.set_children(node)
        self.__cur_node = node

    # Exit a parse tree produced by OpenSCENARIO2Parser#physicalTypeDeclaration.
    def exitPhysicalTypeDeclaration(self, ctx: OpenSCENARIO2Parser.PhysicalTypeDeclarationContext):
        self.__cur_node = self.__node_stack.pop()

    # Enter a parse tree produced by OpenSCENARIO2Parser#physicalTypeName.
    def enterPhysicalTypeName(self, ctx: OpenSCENARIO2Parser.PhysicalTypeNameContext):
        pass

    # Exit a parse tree produced by OpenSCENARIO2Parser#physicalTypeName.
    def exitPhysicalTypeName(self, ctx: OpenSCENARIO2Parser.PhysicalTypeNameContext):
        pass

    # Enter a parse tree produced by OpenSCENARIO2Parser#baseUnitSpecifier.
    def enterBaseUnitSpecifier(self, ctx: OpenSCENARIO2Parser.BaseUnitSpecifierContext):
        pass

    # Exit a parse tree produced by OpenSCENARIO2Parser#baseUnitSpecifier.
    def exitBaseUnitSpecifier(self, ctx: OpenSCENARIO2Parser.BaseUnitSpecifierContext):
        pass

    # Enter a parse tree produced by OpenSCENARIO2Parser#sIBaseUnitSpecifier.
    def enterSIBaseUnitSpecifier(self, ctx: OpenSCENARIO2Parser.SIBaseUnitSpecifierContext):
        pass

    # Exit a parse tree produced by OpenSCENARIO2Parser#sIBaseUnitSpecifier.
    def exitSIBaseUnitSpecifier(self, ctx: OpenSCENARIO2Parser.SIBaseUnitSpecifierContext):
        pass

    # Enter a parse tree produced by OpenSCENARIO2Parser#unitDeclaration.
    def enterUnitDeclaration(self, ctx: OpenSCENARIO2Parser.UnitDeclarationContext):
        self.__node_stack.append(self.__cur_node)
        unit_name = ctx.unitName().getText()

        physical_name = ctx.physicalTypeName().getText()

        node = UnitDeclaration(unit_name, physical_name)
        node.set_ctx(ctx, self.current_file)

        self.__cur_node.set_children(node)
        self.__cur_node = node

    # Exit a parse tree produced by OpenSCENARIO2Parser#unitDeclaration.
    def exitUnitDeclaration(self, ctx: OpenSCENARIO2Parser.UnitDeclarationContext):
        self.__cur_node = self.__node_stack.pop()

    # Enter a parse tree produced by OpenSCENARIO2Parser#unitSpecifier.
    def enterUnitSpecifier(self, ctx: OpenSCENARIO2Parser.UnitSpecifierContext):
        pass

    # Exit a parse tree produced by OpenSCENARIO2Parser#unitSpecifier.
    def exitUnitSpecifier(self, ctx: OpenSCENARIO2Parser.UnitSpecifierContext):
        pass

    # Enter a parse tree produced by OpenSCENARIO2Parser#sIUnitSpecifier.
    def enterSiUnitSpecifier(self, ctx: OpenSCENARIO2Parser.SiUnitSpecifierContext):
        factor = None
        offset = None
        if ctx.siFactor().FloatLiteral():
            factor = float(ctx.siFactor().FloatLiteral().getText())
        elif ctx.siFactor().integerLiteral():
            factor = int(ctx.siFactor().integerLiteral().getText())

        if ctx.siOffset():
            if ctx.siOffset().FloatLiteral():
                offset = float(ctx.siOffset().FloatLiteral().getText())
            elif ctx.siOffset().integerLiteral():
                offset = int(ctx.siOffset().integerLiteral().getText())

        node = SIUnitSpecifier(factor, offset)
        self.__cur_node.set_children(node)

    # Enter a parse tree produced by OpenSCENARIO2Parser#enumDeclaration.
    def enterEnumDeclaration(self, ctx: OpenSCENARIO2Parser.EnumDeclarationContext):
        self.__node_stack.append(self.__cur_node)
        enum_name = ctx.enumName().getText()

        node = EnumDeclaration(enum_name)
        node.set_ctx(ctx, self.current_file)

        self.__cur_node.set_children(node)
        self.__cur_node = node

    # Exit a parse tree produced by OpenSCENARIO2Parser#enumDeclaration.
    def exitEnumDeclaration(self, ctx: OpenSCENARIO2Parser.EnumDeclarationContext):
        self.__cur_node = self.__node_stack.pop()

    # Enter a parse tree produced by OpenSCENARIO2Parser#enumMemberDecl.
    def enterEnumMemberDecl(self, ctx: OpenSCENARIO2Parser.EnumMemberDeclContext):
        self.__node_stack.append(self.__cur_node)
        member_name = ctx.enumMemberName().getText()

        member_value = None
        if ctx.enumMemberValue():
            if ctx.enumMemberValue().UintLiteral():
                member_value = int(ctx.enumMemberValue().UintLiteral().getText())
            elif ctx.enumMemberValue().HexUintLiteral():
                member_value = int(ctx.enumMemberValue().HexUintLiteral().getText(), 16)
            else:
                pass

        # The ModelElement enumeration value is the value stored in the symbol table
        node = EnumMemberDeclaration(member_name, member_value)
        # node.set_ctx(ctx, self.current_file)
        node.set_ctx(ctx, self.current_file)

        self.__cur_node.set_children(node)
        self.__cur_node = node

    # Exit a parse tree produced by OpenSCENARIO2Parser#enumMemberDecl.
    def exitEnumMemberDecl(self, ctx: OpenSCENARIO2Parser.EnumMemberDeclContext):
        self.__cur_node = self.__node_stack.pop()

    # Enter a parse tree produced by OpenSCENARIO2Parser#enumMemberValue.
    def enterEnumMemberValue(self, ctx: OpenSCENARIO2Parser.EnumMemberValueContext):
        pass

    # Exit a parse tree produced by OpenSCENARIO2Parser#enumMemberValue.
    def exitEnumMemberValue(self, ctx: OpenSCENARIO2Parser.EnumMemberValueContext):
        pass

    # Enter a parse tree produced by OpenSCENARIO2Parser#enumName.
    def enterEnumName(self, ctx: OpenSCENARIO2Parser.EnumNameContext):
        pass

    # Exit a parse tree produced by OpenSCENARIO2Parser#enumName.
    def exitEnumName(self, ctx: OpenSCENARIO2Parser.EnumNameContext):
        pass

    # Enter a parse tree produced by OpenSCENARIO2Parser#enumMemberName.
    def enterEnumMemberName(self, ctx: OpenSCENARIO2Parser.EnumMemberNameContext):
        pass

    # Exit a parse tree produced by OpenSCENARIO2Parser#enumMemberName.
    def exitEnumMemberName(self, ctx: OpenSCENARIO2Parser.EnumMemberNameContext):
        pass

    # Enter a parse tree produced by OpenSCENARIO2Parser#enumValueReference.
    def enterEnumValueReference(self, ctx: OpenSCENARIO2Parser.EnumValueReferenceContext):
        self.__node_stack.append(self.__cur_node)
        enum_name = None
        if ctx.enumName():
            enum_name = ctx.enumName().getText()

        enum_member_name = ctx.enumMemberName().getText()

        node = EnumValueReference(enum_name, enum_member_name)
        node.set_ctx(ctx, self.current_file)

        self.__cur_node.set_children(node)
        self.__cur_node = node

    # Exit a parse tree produced by OpenSCENARIO2Parser#enumValueReference.
    def exitEnumValueReference(self, ctx: OpenSCENARIO2Parser.EnumValueReferenceContext):
        self.__cur_node = self.__node_stack.pop()

    # Enter a parse tree produced by OpenSCENARIO2Parser#inheritsCondition.
    def enterInheritsCondition(self, ctx: OpenSCENARIO2Parser.InheritsConditionContext):
        raise OSC2ParsingError(msg=f"inherits condition not yet supported", context=ctx)
        # self.__node_stack.append(self.__cur_node)
        # field_name = ctx.fieldName().getText()

        # # Since there is no function to process bool_literal,
        # # so we manually add a bool literal node to tree.
        # bool_literal_node = None
        # if ctx.BoolLiteral():
        #     bool_literal = ctx.BoolLiteral().getText()
        #     bool_literal_node = BoolLiteral(bool_literal)

        # node = InheritsCondition(field_name, bool_literal_node)
        # node.set_ctx(ctx, self.current_file)

        # self.__cur_node.set_children(node)
        # self.__cur_node = node

    # Exit a parse tree produced by OpenSCENARIO2Parser#inheritsCondition.
    def exitInheritsCondition(self, ctx: OpenSCENARIO2Parser.InheritsConditionContext):
        self.__cur_node = self.__node_stack.pop()
    # Enter a parse tree produced by OpenSCENARIO2Parser#structDeclaration.

    def enterStructDeclaration(self, ctx: OpenSCENARIO2Parser.StructDeclarationContext):
        self.__node_stack.append(self.__cur_node)
        struct_name = ctx.structName().getText()

        node = StructDeclaration(struct_name)
        node.set_ctx(ctx, self.current_file)

        self.__cur_node.set_children(node)
        self.__cur_node = node

    # Exit a parse tree produced by OpenSCENARIO2Parser#structDeclaration.
    def exitStructDeclaration(self, ctx: OpenSCENARIO2Parser.StructDeclarationContext):
        self.__cur_node = self.__node_stack.pop()

    # Enter a parse tree produced by OpenSCENARIO2Parser#structInherits.
    def enterStructInherits(self, ctx: OpenSCENARIO2Parser.StructInheritsContext):
        self.__node_stack.append(self.__cur_node)
        struct_name = ctx.structName().getText()

        node = StructInherits(struct_name)
        node.set_ctx(ctx, self.current_file)

        self.__cur_node.set_children(node)
        self.__cur_node = node

    # Exit a parse tree produced by OpenSCENARIO2Parser#structInherits.
    def exitStructInherits(self, ctx: OpenSCENARIO2Parser.StructInheritsContext):
        self.__cur_node = self.__node_stack.pop()

    # Enter a parse tree produced by OpenSCENARIO2Parser#structMemberDecl.
    def enterStructMemberDecl(self, ctx: OpenSCENARIO2Parser.StructMemberDeclContext):
        pass

    # Exit a parse tree produced by OpenSCENARIO2Parser#structMemberDecl.
    def exitStructMemberDecl(self, ctx: OpenSCENARIO2Parser.StructMemberDeclContext):
        pass

    # Enter a parse tree produced by OpenSCENARIO2Parser#fieldName.
    def enterFieldName(self, ctx: OpenSCENARIO2Parser.FieldNameContext):
        pass

    # Exit a parse tree produced by OpenSCENARIO2Parser#fieldName.
    def exitFieldName(self, ctx: OpenSCENARIO2Parser.FieldNameContext):
        pass

    # Enter a parse tree produced by OpenSCENARIO2Parser#structName.
    def enterStructName(self, ctx: OpenSCENARIO2Parser.StructNameContext):
        pass

    # Exit a parse tree produced by OpenSCENARIO2Parser#structName.
    def exitStructName(self, ctx: OpenSCENARIO2Parser.StructNameContext):
        pass

    # Enter a parse tree produced by OpenSCENARIO2Parser#actorDeclaration.
    def enterActorDeclaration(self, ctx: OpenSCENARIO2Parser.ActorDeclarationContext):
        self.__node_stack.append(self.__cur_node)
        actor_name = ctx.actorName().getText()

        node = ActorDeclaration(actor_name)
        node.set_ctx(ctx, self.current_file)

        self.__cur_node.set_children(node)
        self.__cur_node = node

    # Exit a parse tree produced by OpenSCENARIO2Parser#actorDeclaration.
    def exitActorDeclaration(self, ctx: OpenSCENARIO2Parser.ActorDeclarationContext):
        self.__cur_node = self.__node_stack.pop()

    # Enter a parse tree produced by OpenSCENARIO2Parser#actorInherits.
    def enterActorInherits(self, ctx: OpenSCENARIO2Parser.ActorInheritsContext):
        self.__node_stack.append(self.__cur_node)
        actor_name = ctx.actorName().getText()

        node = ActorInherits(actor_name)
        node.set_ctx(ctx, self.current_file)

        self.__cur_node.set_children(node)
        self.__cur_node = node

    # Exit a parse tree produced by OpenSCENARIO2Parser#actorInherits.
    def exitActorInherits(self, ctx: OpenSCENARIO2Parser.ActorInheritsContext):
        self.__cur_node = self.__node_stack.pop()

    # Enter a parse tree produced by OpenSCENARIO2Parser#actorMemberDecl.
    def enterActorMemberDecl(self, ctx: OpenSCENARIO2Parser.ActorMemberDeclContext):
        pass

    # Exit a parse tree produced by OpenSCENARIO2Parser#actorMemberDecl.
    def exitActorMemberDecl(self, ctx: OpenSCENARIO2Parser.ActorMemberDeclContext):
        pass

    # Enter a parse tree produced by OpenSCENARIO2Parser#actorName.
    def enterActorName(self, ctx: OpenSCENARIO2Parser.ActorNameContext):
        pass

    # Exit a parse tree produced by OpenSCENARIO2Parser#actorName.
    def exitActorName(self, ctx: OpenSCENARIO2Parser.ActorNameContext):
        pass

    # Enter a parse tree produced by OpenSCENARIO2Parser#scenarioDeclaration.
    def enterScenarioDeclaration(self, ctx: OpenSCENARIO2Parser.ScenarioDeclarationContext):
        self.__node_stack.append(self.__cur_node)
        qualified_behavior_name = ctx.qualifiedBehaviorName().getText()

        node = ScenarioDeclaration(qualified_behavior_name)
        node.set_ctx(ctx, self.current_file)

        self.__cur_node.set_children(node)
        self.__cur_node = node

    # Exit a parse tree produced by OpenSCENARIO2Parser#scenarioDeclaration.
    def exitScenarioDeclaration(self, ctx: OpenSCENARIO2Parser.ScenarioDeclarationContext):
        self.__cur_node = self.__node_stack.pop()

    # Enter a parse tree produced by OpenSCENARIO2Parser#scenarioInherits.
    def enterScenarioInherits(self, ctx: OpenSCENARIO2Parser.ScenarioInheritsContext):
        self.__node_stack.append(self.__cur_node)
        qualified_behavior_name = ctx.qualifiedBehaviorName().getText()

        node = ScenarioInherits(qualified_behavior_name)
        node.set_ctx(ctx, self.current_file)

        self.__cur_node.set_children(node)
        self.__cur_node = node

    # Exit a parse tree produced by OpenSCENARIO2Parser#scenarioInherits.
    def exitScenarioInherits(self, ctx: OpenSCENARIO2Parser.ScenarioInheritsContext):
        self.__cur_node = self.__node_stack.pop()

    # Enter a parse tree produced by OpenSCENARIO2Parser#scenarioMemberDecl.
    def enterScenarioMemberDecl(self, ctx: OpenSCENARIO2Parser.ScenarioMemberDeclContext):
        self.__node_stack.append(self.__cur_node)

    # Exit a parse tree produced by OpenSCENARIO2Parser#scenarioMemberDecl.
    def exitScenarioMemberDecl(self, ctx: OpenSCENARIO2Parser.ScenarioMemberDeclContext):
        self.__cur_node = self.__node_stack.pop()

    # Enter a parse tree produced by OpenSCENARIO2Parser#qualifiedBehaviorName.
    def enterQualifiedBehaviorName(self, ctx: OpenSCENARIO2Parser.QualifiedBehaviorNameContext):
        pass

    # Exit a parse tree produced by OpenSCENARIO2Parser#qualifiedBehaviorName.
    def exitQualifiedBehaviorName(self, ctx: OpenSCENARIO2Parser.QualifiedBehaviorNameContext):
        pass

    # Enter a parse tree produced by OpenSCENARIO2Parser#behaviorName.
    def enterBehaviorName(self, ctx: OpenSCENARIO2Parser.BehaviorNameContext):
        pass

    # Exit a parse tree produced by OpenSCENARIO2Parser#behaviorName.
    def exitBehaviorName(self, ctx: OpenSCENARIO2Parser.BehaviorNameContext):
        pass

    # Enter a parse tree produced by OpenSCENARIO2Parser#actionDeclaration.
    def enterActionDeclaration(self, ctx: OpenSCENARIO2Parser.ActionDeclarationContext):
        self.__node_stack.append(self.__cur_node)
        qualified_behavior_name = ctx.qualifiedBehaviorName().getText()
        node = ActionDeclaration(qualified_behavior_name)
        node.set_ctx(ctx, self.current_file)

        self.__cur_node.set_children(node)
        self.__cur_node = node

    # Exit a parse tree produced by OpenSCENARIO2Parser#actionDeclaration.
    def exitActionDeclaration(self, ctx: OpenSCENARIO2Parser.ActionDeclarationContext):
        self.__cur_node = self.__node_stack.pop()

    # Enter a parse tree produced by OpenSCENARIO2Parser#actionInherits.
    def enterActionInherits(self, ctx: OpenSCENARIO2Parser.ActionInheritsContext):
        self.__node_stack.append(self.__cur_node)
        qualified_behavior_name = ctx.qualifiedBehaviorName().getText()

        node = ActionInherits(qualified_behavior_name)
        node.set_ctx(ctx, self.current_file)

        self.__cur_node.set_children(node)
        self.__cur_node = node

    # Exit a parse tree produced by OpenSCENARIO2Parser#actionInherits.
    def exitActionInherits(self, ctx: OpenSCENARIO2Parser.ActionInheritsContext):
        self.__cur_node = self.__node_stack.pop()

    # Enter a parse tree produced by OpenSCENARIO2Parser#modifierDeclaration.
    def enterModifierDeclaration(self, ctx: OpenSCENARIO2Parser.ModifierDeclarationContext):
        raise OSC2ParsingError(msg=f"modifier declaration not supported yet.", context=ctx)
        # self.__node_stack.append(self.__cur_node)
        # actor_name = None
        # if ctx.actorName():
        #     actor_name = ctx.actorName().getText()

        # modifier_name = ctx.modifierName().getText()

        # node = ModifierDeclaration(actor_name, modifier_name)
        # node.set_ctx(ctx, self.current_file)

        # self.__cur_node.set_children(node)
        # self.__cur_node = node

    # Exit a parse tree produced by OpenSCENARIO2Parser#modifierDeclaration.
    def exitModifierDeclaration(self, ctx: OpenSCENARIO2Parser.ModifierDeclarationContext):
        self.__cur_node = self.__node_stack.pop()

    # Enter a parse tree produced by OpenSCENARIO2Parser#modifierName.
    def enterModifierName(self, ctx: OpenSCENARIO2Parser.ModifierNameContext):
        pass

    # Exit a parse tree produced by OpenSCENARIO2Parser#modifierName.
    def exitModifierName(self, ctx: OpenSCENARIO2Parser.ModifierNameContext):
        pass

    # Enter a parse tree produced by OpenSCENARIO2Parser#typeExtension.
    def enterTypeExtension(self, ctx: OpenSCENARIO2Parser.TypeExtensionContext):
        pass

    # Exit a parse tree produced by OpenSCENARIO2Parser#typeExtension.
    def exitTypeExtension(self, ctx: OpenSCENARIO2Parser.TypeExtensionContext):
        pass

    # Enter a parse tree produced by OpenSCENARIO2Parser#enumTypeExtension.
    def enterEnumTypeExtension(self, ctx: OpenSCENARIO2Parser.EnumTypeExtensionContext):
        raise OSC2ParsingError(msg=f"enum type extension not yet supported", context=ctx)
        # self.__node_stack.append(self.__cur_node)
        # enum_name = ctx.enumName().getText()

        # node = EnumTypeExtension(enum_name)
        # node.set_ctx(ctx, self.current_file)

        # self.__cur_node.set_children(node)
        # self.__cur_node = node

    # Exit a parse tree produced by OpenSCENARIO2Parser#enumTypeExtension.
    def exitEnumTypeExtension(self, ctx: OpenSCENARIO2Parser.EnumTypeExtensionContext):
        self.__cur_node = self.__node_stack.pop()

    # Enter a parse tree produced by OpenSCENARIO2Parser#structuredTypeExtension.
    def enterStructuredTypeExtension(self, ctx: OpenSCENARIO2Parser.StructuredTypeExtensionContext):
        self.__node_stack.append(self.__cur_node)

        type_name = None
        if ctx.extendableTypeName().typeName():
            type_name = ctx.extendableTypeName().typeName().getText()
            # Even if a symbol table with the corresponding name is found,
            # it is necessary to determine whether the extended symbol table matches the original type

        # The two ifs here are syntactically mutually exclusive
        qualified_behavior_name = None
        if ctx.extendableTypeName().qualifiedBehaviorName():
            qualified_behavior_name = (
                ctx.extendableTypeName().qualifiedBehaviorName().getText()
            )

        node = StructuredTypeExtension(type_name, qualified_behavior_name)
        node.set_ctx(ctx, self.current_file)

        self.__cur_node.set_children(node)
        self.__cur_node = node

    # Exit a parse tree produced by OpenSCENARIO2Parser#structuredTypeExtension.
    def exitStructuredTypeExtension(self, ctx: OpenSCENARIO2Parser.StructuredTypeExtensionContext):
        self.__cur_node = self.__node_stack.pop()

    # Enter a parse tree produced by OpenSCENARIO2Parser#extendableTypeName.
    def enterExtendableTypeName(self, ctx: OpenSCENARIO2Parser.ExtendableTypeNameContext):
        pass

    # Exit a parse tree produced by OpenSCENARIO2Parser#extendableTypeName.
    def exitExtendableTypeName(self, ctx: OpenSCENARIO2Parser.ExtendableTypeNameContext):
        pass

    # Enter a parse tree produced by OpenSCENARIO2Parser#extensionMemberDecl.
    def enterExtensionMemberDecl(self, ctx: OpenSCENARIO2Parser.ExtensionMemberDeclContext):
        pass

    # Exit a parse tree produced by OpenSCENARIO2Parser#extensionMemberDecl.
    def exitExtensionMemberDecl(self, ctx: OpenSCENARIO2Parser.ExtensionMemberDeclContext):
        pass

    # Enter a parse tree produced by OpenSCENARIO2Parser#globalParameterDeclaration.
    def enterGlobalParameterDeclaration(self, ctx: OpenSCENARIO2Parser.GlobalParameterDeclarationContext):
        default_value = None
        field_type = ctx.typeDeclarator().getText()
        self.__node_stack.append(self.__cur_node)
        if len(ctx.fieldName()) != 1:
            raise OSC2ParsingError(msg="Only single field names are currently supported", context=ctx)
        field_name = ctx.fieldName()[0].Identifier().getText()
        # TODO field_name = []
        # multi_field_name = ""
        # for fn in ctx.fieldName():
        #     name = fn.Identifier().getText()
        #     if name in field_name:
        #         raise OSC2ParsingError(
        #             msg= "Can not define same param in same scope!",
        #             line=ctx.start.line,
        #             column=ctx.start.column,
        #             context=ctx.getText()
        #         )
        #     field_name.append(name)
        #     multi_field_name = multi_field_name_append(multi_field_name, name)

        node = GlobalParameterDeclaration(field_name, field_name, field_type, default_value)
        node.set_ctx(ctx, self.current_file)

        self.__cur_node.set_children(node)
        self.__cur_node = node

    # Exit a parse tree produced by OpenSCENARIO2Parser#globalParameterDeclaration.
    def exitGlobalParameterDeclaration(self, ctx: OpenSCENARIO2Parser.GlobalParameterDeclarationContext):
        self.__cur_node = self.__node_stack.pop()

    # Enter a parse tree produced by OpenSCENARIO2Parser#typeDeclarator.
    def enterTypeDeclarator(self, ctx: OpenSCENARIO2Parser.TypeDeclaratorContext):
        self.__node_stack.append(self.__cur_node)
        type_name = None
        is_list = False
        if ctx.nonAggregateTypeDeclarator():
            type_name = ctx.nonAggregateTypeDeclarator().getText()
            if type_name.startswith('listof'):
                raise OSC2ParsingError(msg=f"Type name starting with listof is not allowed.", context=ctx)
        elif ctx.aggregateTypeDeclarator():
            is_list = True
            type_name = ctx.aggregateTypeDeclarator().getText()

        node = Type(type_name, is_list)

        node.set_ctx(ctx, self.current_file)

        self.__cur_node.set_children(node)
        self.__cur_node = node

    # Exit a parse tree produced by OpenSCENARIO2Parser#typeDeclarator.
    def exitTypeDeclarator(self, ctx: OpenSCENARIO2Parser.TypeDeclaratorContext):
        self.__cur_node = self.__node_stack.pop()

    # Enter a parse tree produced by OpenSCENARIO2Parser#nonAggregateTypeDeclarator.
    def enterNonAggregateTypeDeclarator(self, ctx: OpenSCENARIO2Parser.NonAggregateTypeDeclaratorContext):
        pass

    # Exit a parse tree produced by OpenSCENARIO2Parser#nonAggregateTypeDeclarator.
    def exitNonAggregateTypeDeclarator(self, ctx: OpenSCENARIO2Parser.NonAggregateTypeDeclaratorContext):
        pass

    # Enter a parse tree produced by OpenSCENARIO2Parser#aggregateTypeDeclarator.
    def enterAggregateTypeDeclarator(self, ctx: OpenSCENARIO2Parser.AggregateTypeDeclaratorContext):
        pass

    # Exit a parse tree produced by OpenSCENARIO2Parser#aggregateTypeDeclarator.
    def exitAggregateTypeDeclarator(self, ctx: OpenSCENARIO2Parser.AggregateTypeDeclaratorContext):
        pass

    # Enter a parse tree produced by OpenSCENARIO2Parser#listTypeDeclarator.
    def enterListTypeDeclarator(self, ctx: OpenSCENARIO2Parser.ListTypeDeclaratorContext):
        pass

    # Exit a parse tree produced by OpenSCENARIO2Parser#listTypeDeclarator.
    def exitListTypeDeclarator(self, ctx: OpenSCENARIO2Parser.ListTypeDeclaratorContext):
        pass

    # Enter a parse tree produced by OpenSCENARIO2Parser#primitiveType.
    def enterPrimitiveType(self, ctx: OpenSCENARIO2Parser.PrimitiveTypeContext):
        pass

    # Exit a parse tree produced by OpenSCENARIO2Parser#primitiveType.
    def exitPrimitiveType(self, ctx: OpenSCENARIO2Parser.PrimitiveTypeContext):
        pass

    # Enter a parse tree produced by OpenSCENARIO2Parser#typeName.
    def enterTypeName(self, ctx: OpenSCENARIO2Parser.TypeNameContext):
        pass

    # Exit a parse tree produced by OpenSCENARIO2Parser#typeName.
    def exitTypeName(self, ctx: OpenSCENARIO2Parser.TypeNameContext):
        pass

    # Enter a parse tree produced by OpenSCENARIO2Parser#eventDeclaration.
    def enterEventDeclaration(self, ctx: OpenSCENARIO2Parser.EventDeclarationContext):
        self.__node_stack.append(self.__cur_node)
        event_name = ctx.eventName().getText()

        node = EventDeclaration(event_name)
        node.set_ctx(ctx, self.current_file)

        self.__cur_node.set_children(node)
        self.__cur_node = node

    # Exit a parse tree produced by OpenSCENARIO2Parser#eventDeclaration.
    def exitEventDeclaration(self, ctx: OpenSCENARIO2Parser.EventDeclarationContext):
        self.__cur_node = self.__node_stack.pop()

    # Enter a parse tree produced by OpenSCENARIO2Parser#eventSpecification.
    def enterEventSpecification(self, ctx: OpenSCENARIO2Parser.EventSpecificationContext):
        pass

    # Exit a parse tree produced by OpenSCENARIO2Parser#eventSpecification.
    def exitEventSpecification(self, ctx: OpenSCENARIO2Parser.EventSpecificationContext):
        pass

    # Enter a parse tree produced by OpenSCENARIO2Parser#eventReference.
    def enterEventReference(self, ctx: OpenSCENARIO2Parser.EventReferenceContext):
        self.__node_stack.append(self.__cur_node)
        event_path = ctx.eventPath().getText()
        node = EventReference(event_path)
        node.set_ctx(ctx, self.current_file)

        self.__cur_node.set_children(node)
        self.__cur_node = node

    # Exit a parse tree produced by OpenSCENARIO2Parser#eventReference.
    def exitEventReference(self, ctx: OpenSCENARIO2Parser.EventReferenceContext):
        self.__cur_node = self.__node_stack.pop()

    # Enter a parse tree produced by OpenSCENARIO2Parser#eventFieldDecl.
    def enterEventFieldDecl(self, ctx: OpenSCENARIO2Parser.EventFieldDeclContext):
        self.__node_stack.append(self.__cur_node)
        event_field_name = ctx.eventFieldName().getText()
        node = EventFieldDecl(event_field_name)
        node.set_ctx(ctx, self.current_file)

        self.__cur_node.set_children(node)
        self.__cur_node = node

    # Exit a parse tree produced by OpenSCENARIO2Parser#eventFieldDecl.
    def exitEventFieldDecl(self, ctx: OpenSCENARIO2Parser.EventFieldDeclContext):
        self.__cur_node = self.__node_stack.pop()

    # Enter a parse tree produced by OpenSCENARIO2Parser#eventFieldName.
    def enterEventFieldName(self, ctx: OpenSCENARIO2Parser.EventFieldNameContext):
        pass

    # Exit a parse tree produced by OpenSCENARIO2Parser#eventFieldName.
    def exitEventFieldName(self, ctx: OpenSCENARIO2Parser.EventFieldNameContext):
        pass

    # Enter a parse tree produced by OpenSCENARIO2Parser#eventName.
    def enterEventName(self, ctx: OpenSCENARIO2Parser.EventNameContext):
        pass

    # Exit a parse tree produced by OpenSCENARIO2Parser#eventName.
    def exitEventName(self, ctx: OpenSCENARIO2Parser.EventNameContext):
        pass

    # Enter a parse tree produced by OpenSCENARIO2Parser#eventPath.
    def enterEventPath(self, ctx: OpenSCENARIO2Parser.EventPathContext):
        pass

    # Exit a parse tree produced by OpenSCENARIO2Parser#eventPath.
    def exitEventPath(self, ctx: OpenSCENARIO2Parser.EventPathContext):
        pass

    # Enter a parse tree produced by OpenSCENARIO2Parser#eventCondition.
    def enterEventCondition(self, ctx: OpenSCENARIO2Parser.EventConditionContext):
        self.__node_stack.append(self.__cur_node)
        node = EventCondition()
        node.set_ctx(ctx, self.current_file)

        self.__cur_node.set_children(node)
        self.__cur_node = node

    # Exit a parse tree produced by OpenSCENARIO2Parser#eventCondition.
    def exitEventCondition(self, ctx: OpenSCENARIO2Parser.EventConditionContext):
        self.__cur_node = self.__node_stack.pop()

    # Enter a parse tree produced by OpenSCENARIO2Parser#riseExpression.
    def enterRiseExpression(self, ctx: OpenSCENARIO2Parser.RiseExpressionContext):
        raise OSC2ParsingError(msg=f"rise expression not yet supported", context=ctx)
        # self.__node_stack.append(self.__cur_node)
        # node = RiseExpression()
        # node.set_ctx(ctx, self.current_file)

        # self.__cur_node.set_children(node)
        # self.__cur_node = node

    # Exit a parse tree produced by OpenSCENARIO2Parser#riseExpression.
    def exitRiseExpression(self, ctx: OpenSCENARIO2Parser.RiseExpressionContext):
        self.__cur_node = self.__node_stack.pop()

    # Enter a parse tree produced by OpenSCENARIO2Parser#fallExpression.
    def enterFallExpression(self, ctx: OpenSCENARIO2Parser.FallExpressionContext):
        raise OSC2ParsingError(msg=f"fall expression not yet supported", context=ctx)
        # self.__node_stack.append(self.__cur_node)
        # node = FallExpression()
        # node.set_ctx(ctx, self.current_file)

        # self.__cur_node.set_children(node)
        # self.__cur_node = node

    # Exit a parse tree produced by OpenSCENARIO2Parser#fallExpression.
    def exitFallExpression(self, ctx: OpenSCENARIO2Parser.FallExpressionContext):
        self.__cur_node = self.__node_stack.pop()

    # Enter a parse tree produced by OpenSCENARIO2Parser#elapsedExpression.
    def enterElapsedExpression(self, ctx: OpenSCENARIO2Parser.ElapsedExpressionContext):
        self.__node_stack.append(self.__cur_node)
        node = ElapsedExpression()
        node.set_ctx(ctx, self.current_file)

        self.__cur_node.set_children(node)
        self.__cur_node = node

    # Exit a parse tree produced by OpenSCENARIO2Parser#elapsedExpression.
    def exitElapsedExpression(self, ctx: OpenSCENARIO2Parser.ElapsedExpressionContext):
        self.__cur_node = self.__node_stack.pop()

    # Enter a parse tree produced by OpenSCENARIO2Parser#everyExpression.
    def enterEveryExpression(self, ctx: OpenSCENARIO2Parser.EveryExpressionContext):
        raise OSC2ParsingError(msg=f"every expression not yet supported", context=ctx)
        # self.__node_stack.append(self.__cur_node)
        # node = EveryExpression()
        # node.set_ctx(ctx, self.current_file)

        # self.__cur_node.set_children(node)
        # self.__cur_node = node

    # Exit a parse tree produced by OpenSCENARIO2Parser#everyExpression.
    def exitEveryExpression(self, ctx: OpenSCENARIO2Parser.EveryExpressionContext):
        self.__cur_node = self.__node_stack.pop()

    # Enter a parse tree produced by OpenSCENARIO2Parser#boolExpression.
    def enterBoolExpression(self, ctx: OpenSCENARIO2Parser.BoolExpressionContext):
        pass

    # Exit a parse tree produced by OpenSCENARIO2Parser#boolExpression.
    def exitBoolExpression(self, ctx: OpenSCENARIO2Parser.BoolExpressionContext):
        pass

    # Enter a parse tree produced by OpenSCENARIO2Parser#durationExpression.
    def enterDurationExpression(self, ctx: OpenSCENARIO2Parser.DurationExpressionContext):
        pass

    # Exit a parse tree produced by OpenSCENARIO2Parser#durationExpression.
    def exitDurationExpression(self, ctx: OpenSCENARIO2Parser.DurationExpressionContext):
        pass

    # Enter a parse tree produced by OpenSCENARIO2Parser#fieldDeclaration.
    def enterFieldDeclaration(self, ctx: OpenSCENARIO2Parser.FieldDeclarationContext):
        pass

    # Exit a parse tree produced by OpenSCENARIO2Parser#fieldDeclaration.
    def exitFieldDeclaration(self, ctx: OpenSCENARIO2Parser.FieldDeclarationContext):
        pass

    # Enter a parse tree produced by OpenSCENARIO2Parser#parameterDeclaration.
    def enterParameterDeclaration(self, ctx: OpenSCENARIO2Parser.ParameterDeclarationContext):
        default_value = None
        if ctx.defaultValue():
            default_value = ctx.defaultValue().getText()
        field_type = ctx.typeDeclarator().getText()
        self.__node_stack.append(self.__cur_node)

        if len(ctx.fieldName()) != 1:
            raise OSC2ParsingError(msg="Only single field names are currently supported", context=ctx)
        field_name = ctx.fieldName()[0].Identifier().getText()

        # TODO field_name = []
        # for fn in ctx.fieldName():
        #     name = fn.Identifier().getText()
        #     if name in field_name:
        #         raise OSC2ParsingError(
        #             msg= "Can not define same param in same scope!",
        #             line=ctx.start.line,
        #             column=ctx.start.column,
        #             context=ctx.getText()
        #         )
        #     field_name.append(name)

        node = ParameterDeclaration(field_name, field_type, default_value)
        node.set_ctx(ctx, self.current_file)

        self.__cur_node.set_children(node)
        self.__cur_node = node

    # Exit a parse tree produced by OpenSCENARIO2Parser#parameterDeclaration.
    def exitParameterDeclaration(self, ctx: OpenSCENARIO2Parser.ParameterDeclarationContext):
        self.__cur_node = self.__node_stack.pop()

    # Enter a parse tree produced by OpenSCENARIO2Parser#variableDeclaration.
    def enterVariableDeclaration(self, ctx: OpenSCENARIO2Parser.VariableDeclarationContext):
        self.__node_stack.append(self.__cur_node)
        field_name = []
        default_value = None
        if ctx.sampleExpression():
            default_value = ctx.sampleExpression().getText()
        elif ctx.valueExp():
            if ctx.valueExp().listConstructor():
                default_value = ast.literal_eval(ctx.valueExp().getText())
            else:
                default_value = ctx.valueExp().getText()

        # TODO multi_field_name = ""
        # for fn in ctx.fieldName():
        #     name = fn.Identifier().getText()
        #     if name in field_name:
        #         raise OSC2ParsingError(msg="Can not define same param in same scope!", context=ctx)
        #     field_name.append(name)
        #     multi_field_name = multi_field_name_append(multi_field_name, name)

        field_type = ctx.typeDeclarator().getText()

        node = VariableDeclaration(field_name, field_type, default_value)
        node.set_ctx(ctx, self.current_file)

        self.__cur_node.set_children(node)
        self.__cur_node = node

    # Exit a parse tree produced by OpenSCENARIO2Parser#variableDeclaration.
    def exitVariableDeclaration(self, ctx: OpenSCENARIO2Parser.VariableDeclarationContext):
        self.__cur_node = self.__node_stack.pop()

    # Enter a parse tree produced by OpenSCENARIO2Parser#sampleExpression.
    def enterSampleExpression(self, ctx: OpenSCENARIO2Parser.SampleExpressionContext):
        raise OSC2ParsingError(msg=f"sample expression not yet supported", context=ctx)
        # self.__node_stack.append(self.__cur_node)
        # node = SampleExpression()
        # node.set_ctx(ctx, self.current_file)

        # self.__cur_node.set_children(node)
        # self.__cur_node = node

    # Exit a parse tree produced by OpenSCENARIO2Parser#sampleExpression.
    def exitSampleExpression(self, ctx: OpenSCENARIO2Parser.SampleExpressionContext):
        self.__cur_node = self.__node_stack.pop()

    # Enter a parse tree produced by OpenSCENARIO2Parser#defaultValue.
    def enterDefaultValue(self, ctx: OpenSCENARIO2Parser.DefaultValueContext):
        pass

    # Exit a parse tree produced by OpenSCENARIO2Parser#defaultValue.
    def exitDefaultValue(self, ctx: OpenSCENARIO2Parser.DefaultValueContext):
        pass

    # Enter a parse tree produced by OpenSCENARIO2Parser#parameterWithDeclaration.
    def enterParameterWithDeclaration(self, ctx: OpenSCENARIO2Parser.ParameterWithDeclarationContext):
        pass

    # Exit a parse tree produced by OpenSCENARIO2Parser#parameterWithDeclaration.
    def exitParameterWithDeclaration(self, ctx: OpenSCENARIO2Parser.ParameterWithDeclarationContext):
        pass

    # Enter a parse tree produced by OpenSCENARIO2Parser#parameterWithMember.
    def enterParameterWithMember(self, ctx: OpenSCENARIO2Parser.ParameterWithMemberContext):
        pass

    # Exit a parse tree produced by OpenSCENARIO2Parser#parameterWithMember.
    def exitParameterWithMember(self, ctx: OpenSCENARIO2Parser.ParameterWithMemberContext):
        pass

    # Enter a parse tree produced by OpenSCENARIO2Parser#constraintDeclaration.
    def enterConstraintDeclaration(self, ctx: OpenSCENARIO2Parser.ConstraintDeclarationContext):
        pass

    # Exit a parse tree produced by OpenSCENARIO2Parser#constraintDeclaration.
    def exitConstraintDeclaration(self, ctx: OpenSCENARIO2Parser.ConstraintDeclarationContext):
        pass

    # Enter a parse tree produced by OpenSCENARIO2Parser#keepConstraintDeclaration.
    def enterKeepConstraintDeclaration(self, ctx: OpenSCENARIO2Parser.KeepConstraintDeclarationContext):
        self.__node_stack.append(self.__cur_node)
        constraint_qualifier = None
        if ctx.constraintQualifier():
            constraint_qualifier = ctx.constraintQualifier().getText()

        node = KeepConstraintDeclaration(constraint_qualifier)
        node.set_ctx(ctx, self.current_file)

        self.__cur_node.set_children(node)
        self.__cur_node = node

    # Exit a parse tree produced by OpenSCENARIO2Parser#keepConstraintDeclaration.
    def exitKeepConstraintDeclaration(self, ctx: OpenSCENARIO2Parser.KeepConstraintDeclarationContext):
        self.__cur_node = self.__node_stack.pop()

    # Enter a parse tree produced by OpenSCENARIO2Parser#constraintQualifier.
    def enterConstraintQualifier(self, ctx: OpenSCENARIO2Parser.ConstraintQualifierContext):
        pass

    # Exit a parse tree produced by OpenSCENARIO2Parser#constraintQualifier.
    def exitConstraintQualifier(self, ctx: OpenSCENARIO2Parser.ConstraintQualifierContext):
        pass

    # Enter a parse tree produced by OpenSCENARIO2Parser#constraintExpression.
    def enterConstraintExpression(self, ctx: OpenSCENARIO2Parser.ConstraintExpressionContext):
        pass

    # Exit a parse tree produced by OpenSCENARIO2Parser#constraintExpression.
    def exitConstraintExpression(self, ctx: OpenSCENARIO2Parser.ConstraintExpressionContext):
        pass

    # Enter a parse tree produced by OpenSCENARIO2Parser#removeDefaultDeclaration.
    def enterRemoveDefaultDeclaration(self, ctx: OpenSCENARIO2Parser.RemoveDefaultDeclarationContext):
        raise OSC2ParsingError(msg=f"remove default declaration not supported yet.", context=ctx)
        # self.__node_stack.append(self.__cur_node)
        # node = RemoveDefaultDeclaration()
        # node.set_ctx(ctx, self.current_file)

        # self.__cur_node.set_children(node)
        # self.__cur_node = node

    # Exit a parse tree produced by OpenSCENARIO2Parser#removeDefaultDeclaration.
    def exitRemoveDefaultDeclaration(self, ctx: OpenSCENARIO2Parser.RemoveDefaultDeclarationContext):
        self.__cur_node = self.__node_stack.pop()

    # Enter a parse tree produced by OpenSCENARIO2Parser#parameterReference.
    def enterParameterReference(self, ctx: OpenSCENARIO2Parser.ParameterReferenceContext):
        self.__node_stack.append(self.__cur_node)
        field_name = None
        if ctx.fieldName():
            field_name = ctx.fieldName().getText()

        field_access = None
        if ctx.fieldAccess():
            field_access = ctx.fieldAccess().getText()

        node = ParameterReference(field_name, field_access)
        node.set_ctx(ctx, self.current_file)

        self.__cur_node.set_children(node)
        self.__cur_node = node

    # Exit a parse tree produced by OpenSCENARIO2Parser#parameterReference.
    def exitParameterReference(self, ctx: OpenSCENARIO2Parser.ParameterReferenceContext):
        self.__cur_node = self.__node_stack.pop()

    # Enter a parse tree produced by OpenSCENARIO2Parser#modifierInvocation.
    def enterModifierInvocation(self, ctx: OpenSCENARIO2Parser.ModifierInvocationContext):
        raise OSC2ParsingError(msg=f"modifier invocation not supported yet.", context=ctx)
        # self.__node_stack.append(self.__cur_node)
        # modifier_name = ctx.modifierName().getText()

        # actor = None
        # if ctx.actorExpression():
        #     actor = ctx.actorExpression().getText()

        # if ctx.behaviorExpression():
        #     actor = ctx.behaviorExpression().getText()

        # node = ModifierInvocation(actor, modifier_name)
        # node.set_ctx(ctx, self.current_file)

        # self.__cur_node.set_children(node)
        # self.__cur_node = node

    # Exit a parse tree produced by OpenSCENARIO2Parser#modifierInvocation.
    def exitModifierInvocation(self, ctx: OpenSCENARIO2Parser.ModifierInvocationContext):
        self.__cur_node = self.__node_stack.pop()

    # Enter a parse tree produced by OpenSCENARIO2Parser#behaviorExpression.
    def enterBehaviorExpression(self, ctx: OpenSCENARIO2Parser.BehaviorExpressionContext):
        pass

    # Exit a parse tree produced by OpenSCENARIO2Parser#behaviorExpression.
    def exitBehaviorExpression(self, ctx: OpenSCENARIO2Parser.BehaviorExpressionContext):
        pass

    # Enter a parse tree produced by OpenSCENARIO2Parser#behaviorSpecification.
    def enterBehaviorSpecification(self, ctx: OpenSCENARIO2Parser.BehaviorSpecificationContext):
        pass

    # Exit a parse tree produced by OpenSCENARIO2Parser#behaviorSpecification.
    def exitBehaviorSpecification(self, ctx: OpenSCENARIO2Parser.BehaviorSpecificationContext):
        pass

    # Enter a parse tree produced by OpenSCENARIO2Parser#onDirective.
    def enterOnDirective(self, ctx: OpenSCENARIO2Parser.OnDirectiveContext):
        self.__node_stack.append(self.__cur_node)
        node = OnDirective()
        node.set_ctx(ctx, self.current_file)

        self.__cur_node.set_children(node)
        self.__cur_node = node

    # Exit a parse tree produced by OpenSCENARIO2Parser#onDirective.
    def exitOnDirective(self, ctx: OpenSCENARIO2Parser.OnDirectiveContext):
        self.__cur_node = self.__node_stack.pop()

    # Enter a parse tree produced by OpenSCENARIO2Parser#onMember.
    def enterOnMember(self, ctx: OpenSCENARIO2Parser.OnMemberContext):
        pass

    # Exit a parse tree produced by OpenSCENARIO2Parser#onMember.
    def exitOnMember(self, ctx: OpenSCENARIO2Parser.OnMemberContext):
        pass

    # Enter a parse tree produced by OpenSCENARIO2Parser#doDirective.
    def enterDoDirective(self, ctx: OpenSCENARIO2Parser.DoDirectiveContext):
        self.__node_stack.append(self.__cur_node)

        node = DoDirective()
        node.set_ctx(ctx, self.current_file)

        self.__cur_node.set_children(node)
        self.__cur_node = node

    # Exit a parse tree produced by OpenSCENARIO2Parser#doDirective.
    def exitDoDirective(self, ctx: OpenSCENARIO2Parser.DoDirectiveContext):
        self.__cur_node = self.__node_stack.pop()

    # Enter a parse tree produced by OpenSCENARIO2Parser#doMember.
    def enterDoMember(self, ctx: OpenSCENARIO2Parser.DoMemberContext):
        self.__node_stack.append(self.__cur_node)
        self.__current_label = None

        if ctx.labelName():
            self.__current_label = ctx.labelName().getText()

        composition_operator = None
        if ctx.composition():
            composition_operator = ctx.composition().compositionOperator().getText().strip("\'").strip("\"")

        if composition_operator is not None:
            node = DoMember(self.__current_label, composition_operator)
            node.set_ctx(ctx, self.current_file)

            self.__cur_node.set_children(node)
            self.__cur_node = node

    # Exit a parse tree produced by OpenSCENARIO2Parser#doMember.
    def exitDoMember(self, ctx: OpenSCENARIO2Parser.DoMemberContext):
        self.__cur_node = self.__node_stack.pop()
        self.__current_label = None

    # Enter a parse tree produced by OpenSCENARIO2Parser#composition.
    def enterComposition(self, ctx: OpenSCENARIO2Parser.CompositionContext):
        pass

    # Exit a parse tree produced by OpenSCENARIO2Parser#composition.
    def exitComposition(self, ctx: OpenSCENARIO2Parser.CompositionContext):
        pass

    # Enter a parse tree produced by OpenSCENARIO2Parser#compositionOperator.
    def enterCompositionOperator(self, ctx: OpenSCENARIO2Parser.CompositionOperatorContext):
        pass

    # Exit a parse tree produced by OpenSCENARIO2Parser#compositionOperator.
    def exitCompositionOperator(self, ctx: OpenSCENARIO2Parser.CompositionOperatorContext):
        pass

    # Enter a parse tree produced by OpenSCENARIO2Parser#behaviorInvocation.
    def enterBehaviorInvocation(self, ctx: OpenSCENARIO2Parser.BehaviorInvocationContext):
        self.__node_stack.append(self.__cur_node)
        actor = None
        name = ""
        behavior_name = ctx.behaviorName().getText()
        if ctx.actorExpression():
            actor = ctx.actorExpression().getText()
            name += actor + "."

        name += behavior_name

        node = BehaviorInvocation(self.__current_label, actor, behavior_name)
        node.set_ctx(ctx, self.current_file)

        self.__cur_node.set_children(node)
        self.__cur_node = node

    # Exit a parse tree produced by OpenSCENARIO2Parser#behaviorInvocation.
    def exitBehaviorInvocation(self, ctx: OpenSCENARIO2Parser.BehaviorInvocationContext):
        self.__cur_node = self.__node_stack.pop()

    # Enter a parse tree produced by OpenSCENARIO2Parser#behaviorWithDeclaration.
    def enterBehaviorWithDeclaration(self, ctx: OpenSCENARIO2Parser.BehaviorWithDeclarationContext):
        pass

    # Exit a parse tree produced by OpenSCENARIO2Parser#behaviorWithDeclaration.
    def exitBehaviorWithDeclaration(self, ctx: OpenSCENARIO2Parser.BehaviorWithDeclarationContext):
        pass

    # Enter a parse tree produced by OpenSCENARIO2Parser#behaviorWithMember.
    def enterBehaviorWithMember(self, ctx: OpenSCENARIO2Parser.BehaviorWithMemberContext):
        self.__node_stack.append(self.__cur_node)

    # Exit a parse tree produced by OpenSCENARIO2Parser#behaviorWithMember.
    def exitBehaviorWithMember(self, ctx: OpenSCENARIO2Parser.BehaviorWithMemberContext):
        self.__cur_node = self.__node_stack.pop()

    # Enter a parse tree produced by OpenSCENARIO2Parser#labelName.
    def enterLabelName(self, ctx: OpenSCENARIO2Parser.LabelNameContext):
        pass

    # Exit a parse tree produced by OpenSCENARIO2Parser#labelName.
    def exitLabelName(self, ctx: OpenSCENARIO2Parser.LabelNameContext):
        pass

    # Enter a parse tree produced by OpenSCENARIO2Parser#actorExpression.
    def enterActorExpression(self, ctx: OpenSCENARIO2Parser.ActorExpressionContext):
        pass

    # Exit a parse tree produced by OpenSCENARIO2Parser#actorExpression.
    def exitActorExpression(self, ctx: OpenSCENARIO2Parser.ActorExpressionContext):
        pass

    # Enter a parse tree produced by OpenSCENARIO2Parser#waitDirective.
    def enterWaitDirective(self, ctx: OpenSCENARIO2Parser.WaitDirectiveContext):
        self.__node_stack.append(self.__cur_node)

        node = WaitDirective()
        node.set_ctx(ctx, self.current_file)

        self.__cur_node.set_children(node)
        self.__cur_node = node

    # Exit a parse tree produced by OpenSCENARIO2Parser#waitDirective.
    def exitWaitDirective(self, ctx: OpenSCENARIO2Parser.WaitDirectiveContext):
        self.__cur_node = self.__node_stack.pop()

    # Enter a parse tree produced by OpenSCENARIO2Parser#emitDirective.
    def enterEmitDirective(self, ctx: OpenSCENARIO2Parser.EmitDirectiveContext):
        self.__node_stack.append(self.__cur_node)
        event_name = ctx.eventName().getText()
        node = EmitDirective(event_name)
        node.set_ctx(ctx, self.current_file)

        self.__cur_node.set_children(node)
        self.__cur_node = node

    # Exit a parse tree produced by OpenSCENARIO2Parser#emitDirective.
    def exitEmitDirective(self, ctx: OpenSCENARIO2Parser.EmitDirectiveContext):
        self.__cur_node = self.__node_stack.pop()

    # Enter a parse tree produced by OpenSCENARIO2Parser#callDirective.
    def enterCallDirective(self, ctx: OpenSCENARIO2Parser.CallDirectiveContext):
        raise OSC2ParsingError(msg=f"call directive not supported yet.", context=ctx)
        # self.__node_stack.append(self.__cur_node)
        # method_name = ctx.methodInvocation().postfixExp().getText()

        # node = CallDirective(method_name)
        # node.set_ctx(ctx, self.current_file)

        # self.__cur_node.set_children(node)
        # self.__cur_node = node

    # Exit a parse tree produced by OpenSCENARIO2Parser#callDirective.
    def exitCallDirective(self, ctx: OpenSCENARIO2Parser.CallDirectiveContext):
        self.__cur_node = self.__node_stack.pop()

    # Enter a parse tree produced by OpenSCENARIO2Parser#untilDirective.
    def enterUntilDirective(self, ctx: OpenSCENARIO2Parser.UntilDirectiveContext):
        raise OSC2ParsingError(msg=f"until declaration not supported yet.", context=ctx)
        # self.__node_stack.append(self.__cur_node)
        # node = UntilDirective()
        # node.set_ctx(ctx, self.current_file)

        # self.__cur_node.set_children(node)
        # self.__cur_node = node

    # Exit a parse tree produced by OpenSCENARIO2Parser#untilDirective.
    def exitUntilDirective(self, ctx: OpenSCENARIO2Parser.UntilDirectiveContext):
        self.__cur_node = self.__node_stack.pop()

    # Enter a parse tree produced by OpenSCENARIO2Parser#methodInvocation.
    def enterMethodInvocation(self, ctx: OpenSCENARIO2Parser.MethodInvocationContext):
        pass

    # Exit a parse tree produced by OpenSCENARIO2Parser#methodInvocation.
    def exitMethodInvocation(self, ctx: OpenSCENARIO2Parser.MethodInvocationContext):
        pass

    # Enter a parse tree produced by OpenSCENARIO2Parser#methodDeclaration.
    def enterMethodDeclaration(self, ctx: OpenSCENARIO2Parser.MethodDeclarationContext):
        raise OSC2ParsingError(msg=f"method declaration not supported yet.", context=ctx)
        # self.__node_stack.append(self.__cur_node)
        # method_name = ctx.methodName().getText()
        # return_type = None
        # if ctx.returnType():
        #     return_type = ctx.returnType().getText()

        # node = MethodDeclaration(method_name, return_type)
        # node.set_ctx(ctx, self.current_file)

        # self.__cur_node.set_children(node)
        # self.__cur_node = node

    # Exit a parse tree produced by OpenSCENARIO2Parser#methodDeclaration.
    def exitMethodDeclaration(self, ctx: OpenSCENARIO2Parser.MethodDeclarationContext):
        self.__cur_node = self.__node_stack.pop()

    # Enter a parse tree produced by OpenSCENARIO2Parser#returnType.
    def enterReturnType(self, ctx: OpenSCENARIO2Parser.ReturnTypeContext):
        pass

    # Exit a parse tree produced by OpenSCENARIO2Parser#returnType.
    def exitReturnType(self, ctx: OpenSCENARIO2Parser.ReturnTypeContext):
        pass

    # Enter a parse tree produced by OpenSCENARIO2Parser#methodImplementation.
    def enterMethodImplementation(self, ctx: OpenSCENARIO2Parser.MethodImplementationContext):
        self.__node_stack.append(self.__cur_node)
        qualifier = None
        if ctx.methodQualifier():
            qualifier = ctx.methodQualifier().getText()

        if ctx.expression():
            _type = "expression"
        elif ctx.structuredIdentifier():
            _type = "external"
        else:
            _type = "undefined"

        external_name = None
        if ctx.structuredIdentifier():
            external_name = ctx.structuredIdentifier().getText()

        node = MethodBody(qualifier, _type, external_name)
        node.set_ctx(ctx, self.current_file)

        self.__cur_node.set_children(node)
        self.__cur_node = node

    # Exit a parse tree produced by OpenSCENARIO2Parser#methodImplementation.
    def exitMethodImplementation(self, ctx: OpenSCENARIO2Parser.MethodImplementationContext):
        self.__cur_node = self.__node_stack.pop()

    # Enter a parse tree produced by OpenSCENARIO2Parser#methodQualifier.
    def enterMethodQualifier(self, ctx: OpenSCENARIO2Parser.MethodQualifierContext):
        pass

    # Exit a parse tree produced by OpenSCENARIO2Parser#methodQualifier.
    def exitMethodQualifier(self, ctx: OpenSCENARIO2Parser.MethodQualifierContext):
        pass

    # Enter a parse tree produced by OpenSCENARIO2Parser#methodName.
    def enterMethodName(self, ctx: OpenSCENARIO2Parser.MethodNameContext):
        pass

    # Exit a parse tree produced by OpenSCENARIO2Parser#methodName.
    def exitMethodName(self, ctx: OpenSCENARIO2Parser.MethodNameContext):
        pass

    # Enter a parse tree produced by OpenSCENARIO2Parser#coverageDeclaration.
    def enterCoverageDeclaration(self, ctx: OpenSCENARIO2Parser.CoverageDeclarationContext):
        pass

   # Exit a parse tree produced by OpenSCENARIO2Parser#coverageDeclaration.
    def exitCoverageDeclaration(self, ctx: OpenSCENARIO2Parser.CoverageDeclarationContext):
        pass

    # Enter a parse tree produced by OpenSCENARIO2Parser#coverDeclaration.
    def enterCoverDeclaration(self, ctx: OpenSCENARIO2Parser.CoverDeclarationContext):
        raise OSC2ParsingError(msg=f"cover declaration not supported yet.", context=ctx)
        # self.__node_stack.append(self.__cur_node)
        # target_name = None
        # if ctx.targetName():
        #     target_name = ctx.targetName().getText()

        # node = CoverDeclaration(target_name)
        # node.set_ctx(ctx, self.current_file)

        # self.__cur_node.set_children(node)
        # self.__cur_node = node

    # Exit a parse tree produced by OpenSCENARIO2Parser#coverDeclaration.
    def exitCoverDeclaration(self, ctx: OpenSCENARIO2Parser.CoverDeclarationContext):
        self.__cur_node = self.__node_stack.pop()

    # Enter a parse tree produced by OpenSCENARIO2Parser#recordDeclaration.
    def enterRecordDeclaration(self, ctx: OpenSCENARIO2Parser.RecordDeclarationContext):
        raise OSC2ParsingError(msg=f"record declaration not supported yet.", context=ctx)
        # self.__node_stack.append(self.__cur_node)
        # target_name = None
        # if ctx.targetName():
        #     target_name = ctx.targetName().getText()

        # node = RecordDeclaration(target_name)
        # node.set_ctx(ctx, self.current_file)

        # self.__cur_node.set_children(node)
        # self.__cur_node = node

    # Exit a parse tree produced by OpenSCENARIO2Parser#recordDeclaration.
    def exitRecordDeclaration(self, ctx: OpenSCENARIO2Parser.RecordDeclarationContext):
        self.__cur_node = self.__node_stack.pop()

    # Enter a parse tree produced by OpenSCENARIO2Parser#coverageExpression.
    def enterCoverageExpression(self, ctx: OpenSCENARIO2Parser.CoverageExpressionContext):
        self.__node_stack.append(self.__cur_node)
        argument_name = "expression"
        node = NamedArgument(argument_name)
        node.set_ctx(ctx, self.current_file)

        self.__cur_node.set_children(node)
        self.__cur_node = node

    # Exit a parse tree produced by OpenSCENARIO2Parser#coverageExpression.
    def exitCoverageExpression(self, ctx: OpenSCENARIO2Parser.CoverageExpressionContext):
        self.__cur_node = self.__node_stack.pop()

    # Enter a parse tree produced by OpenSCENARIO2Parser#coverageUnit.
    def enterCoverageUnit(self, ctx: OpenSCENARIO2Parser.CoverageUnitContext):
        self.__node_stack.append(self.__cur_node)
        argument_name = "unit"
        node = NamedArgument(argument_name)
        node.set_ctx(ctx, self.current_file)

        unit_name = Identifier(ctx.Identifier().getText())
        unit_name.set_loc(ctx.start.line, ctx.start.column)
        node.set_children(unit_name)

        self.__cur_node.set_children(node)
        self.__cur_node = node

    # Exit a parse tree produced by OpenSCENARIO2Parser#coverageUnit.
    def exitCoverageUnit(self, ctx: OpenSCENARIO2Parser.CoverageUnitContext):
        self.__cur_node = self.__node_stack.pop()

    # Enter a parse tree produced by OpenSCENARIO2Parser#coverageRange.
    def enterCoverageRange(self, ctx: OpenSCENARIO2Parser.CoverageRangeContext):
        self.__node_stack.append(self.__cur_node)
        argument_name = "range"
        node = NamedArgument(argument_name)
        node.set_ctx(ctx, self.current_file)

        self.__cur_node.set_children(node)
        self.__cur_node = node

    # Exit a parse tree produced by OpenSCENARIO2Parser#coverageRange.
    def exitCoverageRange(self, ctx: OpenSCENARIO2Parser.CoverageRangeContext):
        self.__cur_node = self.__node_stack.pop()

    # Enter a parse tree produced by OpenSCENARIO2Parser#coverageEvery.
    def enterCoverageEvery(self, ctx: OpenSCENARIO2Parser.CoverageEveryContext):
        self.__node_stack.append(self.__cur_node)
        argument_name = "every"
        node = NamedArgument(argument_name)
        node.set_ctx(ctx, self.current_file)

        self.__cur_node.set_children(node)
        self.__cur_node = node

    # Exit a parse tree produced by OpenSCENARIO2Parser#coverageEvery.
    def exitCoverageEvery(self, ctx: OpenSCENARIO2Parser.CoverageEveryContext):
        self.__cur_node = self.__node_stack.pop()

    # Enter a parse tree produced by OpenSCENARIO2Parser#coverageEvent.
    def enterCoverageEvent(self, ctx: OpenSCENARIO2Parser.CoverageEventContext):
        self.__node_stack.append(self.__cur_node)
        argument_name = "event"
        node = NamedArgument(argument_name)
        node.set_ctx(ctx, self.current_file)

        self.__cur_node.set_children(node)
        self.__cur_node = node

    # Exit a parse tree produced by OpenSCENARIO2Parser#coverageEvent.
    def exitCoverageEvent(self, ctx: OpenSCENARIO2Parser.CoverageEventContext):
        self.__cur_node = self.__node_stack.pop()

    # Enter a parse tree produced by OpenSCENARIO2Parser#coverageNameArgument.
    def enterCoverageNameArgument(self, ctx: OpenSCENARIO2Parser.CoverageNameArgumentContext):
        pass

    # Exit a parse tree produced by OpenSCENARIO2Parser#coverageNameArgument.
    def exitCoverageNameArgument(self, ctx: OpenSCENARIO2Parser.CoverageNameArgumentContext):
        pass

    # Enter a parse tree produced by OpenSCENARIO2Parser#targetName.
    def enterTargetName(self, ctx: OpenSCENARIO2Parser.TargetNameContext):
        pass

    # Exit a parse tree produced by OpenSCENARIO2Parser#targetName.
    def exitTargetName(self, ctx: OpenSCENARIO2Parser.TargetNameContext):
        pass

    # Enter a parse tree produced by OpenSCENARIO2Parser#expression.
    def enterExpression(self, ctx: OpenSCENARIO2Parser.ExpressionContext):
        pass

    # Exit a parse tree produced by OpenSCENARIO2Parser#expression.
    def exitExpression(self, ctx: OpenSCENARIO2Parser.ExpressionContext):
        pass

    # Enter a parse tree produced by OpenSCENARIO2Parser#ternaryOpExp.
    def enterTernaryOpExp(self, ctx: OpenSCENARIO2Parser.TernaryOpExpContext):
        pass

    # Exit a parse tree produced by OpenSCENARIO2Parser#ternaryOpExp.
    def exitTernaryOpExp(self, ctx: OpenSCENARIO2Parser.TernaryOpExpContext):
        pass

    # Enter a parse tree produced by OpenSCENARIO2Parser#implication.
    def enterImplication(self, ctx: OpenSCENARIO2Parser.ImplicationContext):
        self.__node_stack.append(self.__cur_node)
        if len(ctx.disjunction()) > 1:
            operator = "=>"
            node = LogicalExpression(operator)
            node.set_ctx(ctx, self.current_file)

            self.__cur_node.set_children(node)
            self.__cur_node = node

    # Exit a parse tree produced by OpenSCENARIO2Parser#implication.
    def exitImplication(self, ctx: OpenSCENARIO2Parser.ImplicationContext):
        self.__cur_node = self.__node_stack.pop()

    # Enter a parse tree produced by OpenSCENARIO2Parser#disjunction.
    def enterDisjunction(self, ctx: OpenSCENARIO2Parser.DisjunctionContext):
        self.__node_stack.append(self.__cur_node)
        if len(ctx.conjunction()) > 1:
            operator = "or"
            node = LogicalExpression(operator)
            node.set_ctx(ctx, self.current_file)

            self.__cur_node.set_children(node)
            self.__cur_node = node

    # Exit a parse tree produced by OpenSCENARIO2Parser#disjunction.
    def exitDisjunction(self, ctx: OpenSCENARIO2Parser.DisjunctionContext):
        self.__cur_node = self.__node_stack.pop()

    # Enter a parse tree produced by OpenSCENARIO2Parser#conjunction.
    def enterConjunction(self, ctx: OpenSCENARIO2Parser.ConjunctionContext):
        self.__node_stack.append(self.__cur_node)
        if len(ctx.inversion()) > 1:
            operator = "and"
            node = LogicalExpression(operator)
            node.set_ctx(ctx, self.current_file)

            self.__cur_node.set_children(node)
            self.__cur_node = node

    # Exit a parse tree produced by OpenSCENARIO2Parser#conjunction.
    def exitConjunction(self, ctx: OpenSCENARIO2Parser.ConjunctionContext):
        self.__cur_node = self.__node_stack.pop()

    # Enter a parse tree produced by OpenSCENARIO2Parser#inversion.
    def enterInversion(self, ctx: OpenSCENARIO2Parser.InversionContext):
        self.__node_stack.append(self.__cur_node)
        if ctx.relation() is None:
            operator = "not"
            node = LogicalExpression(operator)
            node.set_ctx(ctx, self.current_file)

            self.__cur_node.set_children(node)
            self.__cur_node = node

    # Exit a parse tree produced by OpenSCENARIO2Parser#inversion.
    def exitInversion(self, ctx: OpenSCENARIO2Parser.InversionContext):
        self.__cur_node = self.__node_stack.pop()

    # Enter a parse tree produced by OpenSCENARIO2Parser#relation.
    def enterRelation(self, ctx: OpenSCENARIO2Parser.RelationContext):  # pylint: disable=invalid-name
        self.__node_stack.append(self.__cur_node)

    # Exit a parse tree produced by OpenSCENARIO2Parser#relation.
    def exitRelation(self, ctx: OpenSCENARIO2Parser.RelationContext):  # pylint: disable=invalid-name
        self.__cur_node = self.__node_stack.pop()

    # Enter a parse tree produced by OpenSCENARIO2Parser#relationExp.
    def enterRelationExp(self, ctx: OpenSCENARIO2Parser.RelationExpContext):
        self.__node_stack.append(self.__cur_node)
        operator = ctx.relationalOp().getText()
        node = RelationExpression(operator)
        node.set_ctx(ctx, self.current_file)

        self.__cur_node.set_children(node)
        self.__cur_node = node

    # Exit a parse tree produced by OpenSCENARIO2Parser#relationExp.
    def exitRelationExp(self, ctx: OpenSCENARIO2Parser.RelationExpContext):
        self.__cur_node = self.__node_stack.pop()

    # Enter a parse tree produced by OpenSCENARIO2Parser#relationalOp.
    def enterRelationalOp(self, ctx: OpenSCENARIO2Parser.RelationalOpContext):
        pass

    # Exit a parse tree produced by OpenSCENARIO2Parser#relationalOp.
    def exitRelationalOp(self, ctx: OpenSCENARIO2Parser.RelationalOpContext):
        pass

    # Enter a parse tree produced by OpenSCENARIO2Parser#sum.
    def enterSumExpression(self, ctx: OpenSCENARIO2Parser.SumExpressionContext):  # pylint: disable=invalid-name
        pass

    # Exit a parse tree produced by OpenSCENARIO2Parser#sum.
    def exitSumExpression(self, ctx: OpenSCENARIO2Parser.SumExpressionContext):  # pylint: disable=invalid-name
        pass

    # Enter a parse tree produced by OpenSCENARIO2Parser#additiveExp.
    def enterAdditiveExp(self, ctx: OpenSCENARIO2Parser.AdditiveExpContext):
        self.__node_stack.append(self.__cur_node)
        operator = ctx.additiveOp().getText()
        node = BinaryExpression(operator)
        node.set_ctx(ctx, self.current_file)

        self.__cur_node.set_children(node)
        self.__cur_node = node

    # Exit a parse tree produced by OpenSCENARIO2Parser#additiveExp.
    def exitAdditiveExp(self, ctx: OpenSCENARIO2Parser.AdditiveExpContext):
        self.__cur_node = self.__node_stack.pop()

    # Enter a parse tree produced by OpenSCENARIO2Parser#additiveOp.
    def enterAdditiveOp(self, ctx: OpenSCENARIO2Parser.AdditiveOpContext):
        pass

    # Exit a parse tree produced by OpenSCENARIO2Parser#additiveOp.
    def exitAdditiveOp(self, ctx: OpenSCENARIO2Parser.AdditiveOpContext):
        pass

    # Enter a parse tree produced by OpenSCENARIO2Parser#multiplicativeExp.
    def enterMultiplicativeExp(self, ctx: OpenSCENARIO2Parser.MultiplicativeExpContext):
        self.__node_stack.append(self.__cur_node)
        operator = ctx.multiplicativeOp().getText()
        node = BinaryExpression(operator)
        node.set_ctx(ctx, self.current_file)

        self.__cur_node.set_children(node)
        self.__cur_node = node

    # Exit a parse tree produced by OpenSCENARIO2Parser#multiplicativeExp.
    def exitMultiplicativeExp(self, ctx: OpenSCENARIO2Parser.MultiplicativeExpContext):
        self.__cur_node = self.__node_stack.pop()

    # Enter a parse tree produced by OpenSCENARIO2Parser#term.
    def enterTerm(self, ctx: OpenSCENARIO2Parser.TermContext):  # pylint: disable=invalid-name
        pass

    # Exit a parse tree produced by OpenSCENARIO2Parser#term.
    def exitTerm(self, ctx: OpenSCENARIO2Parser.TermContext):  # pylint: disable=invalid-name
        pass

    # Enter a parse tree produced by OpenSCENARIO2Parser#multiplicativeOp.
    def enterMultiplicativeOp(self, ctx: OpenSCENARIO2Parser.MultiplicativeOpContext):
        pass

    # Exit a parse tree produced by OpenSCENARIO2Parser#multiplicativeOp.
    def exitMultiplicativeOp(self, ctx: OpenSCENARIO2Parser.MultiplicativeOpContext):
        pass

    # Enter a parse tree produced by OpenSCENARIO2Parser#factor.
    def enterFactor(self, ctx: OpenSCENARIO2Parser.FactorContext):
        pass

    # Exit a parse tree produced by OpenSCENARIO2Parser#factor.
    def exitFactor(self, ctx: OpenSCENARIO2Parser.FactorContext):
        pass

    # Enter a parse tree produced by OpenSCENARIO2Parser#primaryExpression.
    def enterPrimaryExpression(self, ctx: OpenSCENARIO2Parser.PrimaryExpressionContext):
        pass

    # Exit a parse tree produced by OpenSCENARIO2Parser#primaryExpression.
    def exitPrimaryExpression(self, ctx: OpenSCENARIO2Parser.PrimaryExpressionContext):
        pass

    # Enter a parse tree produced by OpenSCENARIO2Parser#castExpression.
    def enterCastExpression(self, ctx: OpenSCENARIO2Parser.CastExpressionContext):
        raise OSC2ParsingError(msg=f"cast expression not yet supported", context=ctx)
        # self.__node_stack.append(self.__cur_node)
        # object = ctx.postfixExp().getText()
        # target_type = ctx.typeDeclarator().getText()
        # node = CastExpression(object, target_type)
        # node.set_ctx(ctx, self.current_file)

        # self.__cur_node.set_children(node)
        # self.__cur_node = node

    # Exit a parse tree produced by OpenSCENARIO2Parser#castExpression.
    def exitCastExpression(self, ctx: OpenSCENARIO2Parser.CastExpressionContext):
        self.__cur_node = self.__node_stack.pop()

    # Enter a parse tree produced by OpenSCENARIO2Parser#functionApplicationExpression.
    def enterFunctionApplicationExpression(self, ctx: OpenSCENARIO2Parser.FunctionApplicationExpressionContext):
        self.__node_stack.append(self.__cur_node)
        func_name = ctx.postfixExp().getText()

        node = FunctionApplicationExpression(func_name)
        node.set_ctx(ctx, self.current_file)

        self.__cur_node.set_children(node)
        self.__cur_node = node

    # Exit a parse tree produced by OpenSCENARIO2Parser#functionApplicationExpression.
    def exitFunctionApplicationExpression(self, ctx: OpenSCENARIO2Parser.FunctionApplicationExpressionContext):
        self.__cur_node = self.__node_stack.pop()

    # Enter a parse tree produced by OpenSCENARIO2Parser#fieldAccessExpression.
    def enterFieldAccessExpression(self, ctx: OpenSCENARIO2Parser.FieldAccessExpressionContext):
        self.__node_stack.append(self.__cur_node)
        field_name = ctx.postfixExp().getText() + "." + ctx.fieldName().getText()

        node = FieldAccessExpression(field_name)
        node.set_ctx(ctx, self.current_file)

        self.__cur_node.set_children(node)
        self.__cur_node = node

    # Exit a parse tree produced by OpenSCENARIO2Parser#fieldAccessExpression.
    def exitFieldAccessExpression(self, ctx: OpenSCENARIO2Parser.FieldAccessExpressionContext):
        self.__cur_node = self.__node_stack.pop()

    # Enter a parse tree produced by OpenSCENARIO2Parser#elementAccessExpression.
    def enterElementAccessExpression(self, ctx: OpenSCENARIO2Parser.ElementAccessExpressionContext):
        raise OSC2ParsingError(msg=f"element access expression not yet supported", context=ctx)
        # self.__node_stack.append(self.__cur_node)
        # list_name = ctx.postfixExp().getText()
        # index = ctx.expression().getText()
        # node = ElementAccessExpression(list_name, index)
        # node.set_ctx(ctx, self.current_file)

        # self.__cur_node.set_children(node)
        # self.__cur_node = node

    # Exit a parse tree produced by OpenSCENARIO2Parser#elementAccessExpression.
    def exitElementAccessExpression(self, ctx: OpenSCENARIO2Parser.ElementAccessExpressionContext):
        self.__cur_node = self.__node_stack.pop()

    # Enter a parse tree produced by OpenSCENARIO2Parser#typeTestExpression.
    def enterTypeTestExpression(self, ctx: OpenSCENARIO2Parser.TypeTestExpressionContext):
        raise OSC2ParsingError(msg=f"type test expression not yet supported", context=ctx)
        # self.__node_stack.append(self.__cur_node)
        # object = ctx.postfixExp().getText()
        # target_type = ctx.typeDeclarator().getText()
        # node = TypeTestExpression(object, target_type)
        # node.set_ctx(ctx, self.current_file)

        # self.__cur_node.set_children(node)
        # self.__cur_node = node

    # Exit a parse tree produced by OpenSCENARIO2Parser#typeTestExpression.
    def exitTypeTestExpression(self, ctx: OpenSCENARIO2Parser.TypeTestExpressionContext):
        self.__cur_node = self.__node_stack.pop()

    # Enter a parse tree produced by OpenSCENARIO2Parser#fieldAccess.
    def enterFieldAccess(self, ctx: OpenSCENARIO2Parser.FieldAccessContext):
        self.__node_stack.append(self.__cur_node)
        field_name = ctx.postfixExp().getText() + "." + ctx.fieldName().getText()
        node = FieldAccessExpression(field_name)
        node.set_ctx(ctx, self.current_file)

        self.__cur_node.set_children(node)
        self.__cur_node = node

    # Exit a parse tree produced by OpenSCENARIO2Parser#fieldAccess.
    def exitFieldAccess(self, ctx: OpenSCENARIO2Parser.FieldAccessContext):
        self.__cur_node = self.__node_stack.pop()

    # Enter a parse tree produced by OpenSCENARIO2Parser#primaryExp.
    def enterPrimaryExp(self, ctx: OpenSCENARIO2Parser.PrimaryExpContext):
        pass

    # Exit a parse tree produced by OpenSCENARIO2Parser#primaryExp.
    def exitPrimaryExp(self, ctx: OpenSCENARIO2Parser.PrimaryExpContext):
        pass

    # Enter a parse tree produced by OpenSCENARIO2Parser#valueExp.
    def enterValueExp(self, ctx: OpenSCENARIO2Parser.ValueExpContext):
        self.__node_stack.append(self.__cur_node)
        value = None
        node = None
        if ctx.FloatLiteral():
            value = ctx.FloatLiteral().getText()
            node = FloatLiteral(value)
        elif ctx.BoolLiteral():
            value = ctx.BoolLiteral().getText()
            node = BoolLiteral(value)
        elif ctx.StringLiteral():
            value = ctx.StringLiteral().getText()
            value = value.strip('"')
            node = StringLiteral(value)

        if node is not None:
            node.set_ctx(ctx, self.current_file)

            self.__cur_node.set_children(node)
            self.__cur_node = node

    # Exit a parse tree produced by OpenSCENARIO2Parser#valueExp.
    def exitValueExp(self, ctx: OpenSCENARIO2Parser.ValueExpContext):
        self.__cur_node = self.__node_stack.pop()

    # Enter a parse tree produced by OpenSCENARIO2Parser#listConstructor.
    def enterListConstructor(self, ctx: OpenSCENARIO2Parser.ListConstructorContext):
        self.__node_stack.append(self.__cur_node)
        node = ListExpression()
        node.set_ctx(ctx, self.current_file)

        self.__cur_node.set_children(node)
        self.__cur_node = node

    # Exit a parse tree produced by OpenSCENARIO2Parser#listConstructor.
    def exitListConstructor(self, ctx: OpenSCENARIO2Parser.ListConstructorContext):
        self.__cur_node = self.__node_stack.pop()

    # Enter a parse tree produced by OpenSCENARIO2Parser#rangeConstructor.
    def enterRangeConstructor(self, ctx: OpenSCENARIO2Parser.RangeConstructorContext):
        raise OSC2ParsingError(msg=f"range constructor not yet supported", context=ctx)
        # self.__node_stack.append(self.__cur_node)
        # node = RangeExpression()
        # node.set_ctx(ctx, self.current_file)

        # self.__cur_node.set_children(node)
        # self.__cur_node = node

    # Exit a parse tree produced by OpenSCENARIO2Parser#rangeConstructor.
    def exitRangeConstructor(self, ctx: OpenSCENARIO2Parser.RangeConstructorContext):
        self.__cur_node = self.__node_stack.pop()

    # Enter a parse tree produced by OpenSCENARIO2Parser#identifierReference.
    def enterIdentifierReference(self, ctx: OpenSCENARIO2Parser.IdentifierReferenceContext):
        self.__node_stack.append(self.__cur_node)

        field_name = []
        for fn in ctx.fieldName():
            name = fn.Identifier().getText()

            field_name.append(name)

        id_name = ".".join(field_name)  # TODO do not merge to single string

        node = IdentifierReference(id_name)
        node.set_ctx(ctx, self.current_file)

        self.__cur_node.set_children(node)
        self.__cur_node = node

    # Exit a parse tree produced by OpenSCENARIO2Parser#identifierReference.
    def exitIdentifierReference(self, ctx: OpenSCENARIO2Parser.IdentifierReferenceContext):
        self.__cur_node = self.__node_stack.pop()

    # Enter a parse tree produced by OpenSCENARIO2Parser#argumentListSpecification.
    def enterArgumentListSpecification(self, ctx: OpenSCENARIO2Parser.ArgumentListSpecificationContext):
        pass

    # Exit a parse tree produced by OpenSCENARIO2Parser#argumentListSpecification.
    def exitArgumentListSpecification(self, ctx: OpenSCENARIO2Parser.ArgumentListSpecificationContext):
        pass

    # Enter a parse tree produced by OpenSCENARIO2Parser#argumentSpecification.
    def enterArgumentSpecification(self, ctx: OpenSCENARIO2Parser.ArgumentSpecificationContext):
        self.__node_stack.append(self.__cur_node)
        argument_name = ctx.argumentName().getText()
        argument_type = ctx.typeDeclarator().getText()
        default_value = None
        if ctx.defaultValue():
            default_value = ctx.defaultValue().getText()

        node = Argument(argument_name, argument_type, default_value)
        node.set_ctx(ctx, self.current_file)

        self.__cur_node.set_children(node)
        self.__cur_node = node

    # Exit a parse tree produced by OpenSCENARIO2Parser#argumentSpecification.
    def exitArgumentSpecification(self, ctx: OpenSCENARIO2Parser.ArgumentSpecificationContext):
        self.__cur_node = self.__node_stack.pop()

    # Enter a parse tree produced by OpenSCENARIO2Parser#argumentName.
    def enterArgumentName(self, ctx: OpenSCENARIO2Parser.ArgumentNameContext):
        pass

    # Exit a parse tree produced by OpenSCENARIO2Parser#argumentName.
    def exitArgumentName(self, ctx: OpenSCENARIO2Parser.ArgumentNameContext):
        pass

    # Enter a parse tree produced by OpenSCENARIO2Parser#argumentList.
    def enterArgumentList(self, ctx: OpenSCENARIO2Parser.ArgumentListContext):
        pass

    # Exit a parse tree produced by OpenSCENARIO2Parser#argumentList.
    def exitArgumentList(self, ctx: OpenSCENARIO2Parser.ArgumentListContext):
        pass

    # Enter a parse tree produced by OpenSCENARIO2Parser#positionalArgument.
    def enterPositionalArgument(self, ctx: OpenSCENARIO2Parser.PositionalArgumentContext):
        self.__node_stack.append(self.__cur_node)

        node = PositionalArgument()
        node.set_ctx(ctx, self.current_file)

        self.__cur_node.set_children(node)
        self.__cur_node = node

    # Exit a parse tree produced by OpenSCENARIO2Parser#positionalArgument.
    def exitPositionalArgument(self, ctx: OpenSCENARIO2Parser.PositionalArgumentContext):
        self.__cur_node = self.__node_stack.pop()

    # Enter a parse tree produced by OpenSCENARIO2Parser#namedArgument.
    def enterNamedArgument(self, ctx: OpenSCENARIO2Parser.NamedArgumentContext):
        self.__node_stack.append(self.__cur_node)
        argument_name = ctx.argumentName().getText()

        node = NamedArgument(argument_name)
        node.set_ctx(ctx, self.current_file)

        self.__cur_node.set_children(node)
        self.__cur_node = node

    # Exit a parse tree produced by OpenSCENARIO2Parser#namedArgument.
    def exitNamedArgument(self, ctx: OpenSCENARIO2Parser.NamedArgumentContext):
        self.__cur_node = self.__node_stack.pop()

    # Enter a parse tree produced by OpenSCENARIO2Parser#physicalLiteral.
    def enterPhysicalLiteral(self, ctx: OpenSCENARIO2Parser.PhysicalLiteralContext):
        self.__node_stack.append(self.__cur_node)
        unit_name = ctx.unitName().getText()
        value = None
        if ctx.FloatLiteral():
            value = ctx.FloatLiteral().getText()
        else:
            value = ctx.integerLiteral().getText()

        node = PhysicalLiteral(unit_name, value)
        node.set_ctx(ctx, self.current_file)

        self.__cur_node.set_children(node)
        self.__cur_node = node

        if ctx.FloatLiteral():
            self.__node_stack.append(self.__cur_node)
            float_value = ctx.FloatLiteral().getText()
            value_node = FloatLiteral(float_value)
            value_node.set_ctx(ctx, self.current_file)

            node.set_children(value_node)
            self.__cur_node = self.__node_stack.pop()

    # Exit a parse tree produced by OpenSCENARIO2Parser#physicalLiteral.
    def exitPhysicalLiteral(self, ctx: OpenSCENARIO2Parser.PhysicalLiteralContext):
        self.__cur_node = self.__node_stack.pop()

    # Enter a parse tree produced by OpenSCENARIO2Parser#integerLiteral.
    def enterIntegerLiteral(self, ctx: OpenSCENARIO2Parser.IntegerLiteralContext):
        self.__node_stack.append(self.__cur_node)
        value = None
        type_def = "uint"
        if ctx.UintLiteral():
            value = int(ctx.UintLiteral().getText())
            type_def = "uint"
        elif ctx.HexUintLiteral():
            value = int(ctx.HexUintLiteral().getText(), 16)
            type_def = "uint"
        elif ctx.IntLiteral():
            value = int(ctx.IntLiteral().getText())
            type_def = "int"
        else:  # only the above three types of integer literal
            raise ValueError("invalid literal")

        node = IntegerLiteral(type_def, value)
        node.set_ctx(ctx, self.current_file)

        self.__cur_node.set_children(node)
        self.__cur_node = node

    # Exit a parse tree produced by OpenSCENARIO2Parser#integerLiteral.
    def exitIntegerLiteral(self, ctx: OpenSCENARIO2Parser.IntegerLiteralContext):
        self.__cur_node = self.__node_stack.pop()


del OpenSCENARIO2Parser
