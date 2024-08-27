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

from typing import List
from scenario_execution.model.error import OSC2ParsingError
import sys
import py_trees
import operator as op


def print_tree(elem, logger, whitespace=""):

    children = ""
    for child in elem.get_children():
        if not isinstance(child, ModelElement):
            children += f"{child} "

    logger.info(f"{whitespace}{elem}{children}")

    for child in elem.get_children():
        if isinstance(child, ModelElement):
            print_tree(child, logger, whitespace + "  ")


def to_string(elem):
    if isinstance(elem, (str, int, float, bool)):
        return str(elem)
    elif isinstance(elem, StringLiteral):
        return elem.value
    output = f"{elem.__class__.__name__}("
    first = True
    for attr in vars(elem):
        if not attr.startswith('_') and attr != "iter":
            attr_val = getattr(elem, attr)
            if not attr_val:
                continue
            if first:
                first = False
            else:
                output += ", "
            output += attr

            output += "=" + to_string(attr_val)
    if elem.get_child_count():
        if not first:
            output += ", "
        output += "children=["
        first = True
    for child in elem.get_children():
        if first:
            first = False
        else:
            output += ", "
        output += to_string(child)
    if elem.get_child_count():
        output += "]"
    output += ")"
    return output


def serialize(elem):
    result = {}
    children = elem.get_children()
    if children:
        result[type(elem).__name__] = {}
        args = elem.__init__.__code__.co_varnames[1:]
        for arg in args:
            try:
                result[type(elem).__name__][arg] = elem.__dict__[arg]
            except KeyError as e:
                raise KeyError(f"Invalid key for element {type(elem).__name__}: {e}") from e
        result[type(elem).__name__]['_children'] = []
        for child in children:
            result[type(elem).__name__]['_children'].append(serialize(child))
    else:
        result[type(elem).__name__] = {}
    return result


def deserialize(elem):
    types_module = sys.modules[__name__]
    result = getattr(types_module, 'CompilationUnit')()
    for current in elem:
        for key, value in current.items():
            elem_attributes = {k: v for k, v in value.items() if k != '_children'}
            module = getattr(types_module, key)
            inst = module(**elem_attributes)
            if '_children' in value:
                children = deserialize(value['_children'])
                for child_node in children.get_children():
                    inst.set_children(child_node)
            result.set_children(inst)
    return result


class ModelElement(object):  # pylint: disable=too-many-public-methods
    def __init__(self, name=""):
        self.__context = None  # For error logging only
        self.__line = None
        self.__column = None
        self.__filename = None
        self.__children = []  # child node
        self.__parent = None
        self.name = name
        self.iter = None

    def get_value_child(self):
        return None
        # raise NotImplementedError()

    def get_child_count(self):
        return len(self.__children)

    def get_children(self):
        if self.__children is not None:
            yield from self.__children

    def get_child(self, i):
        return self.__children[i]

    def get_only_child(self):
        if self.get_child_count() != 1:
            raise ValueError(f"Expected only one child. Found {self.get_child_count()}")
        return self.get_child(0)

    def set_children(self, *children):
        for child in children:
            if child is not None:
                if isinstance(child, List):
                    for ch in child:
                        if isinstance(ch, ModelElement):
                            ch.__parent = self  # pylint: disable=protected-access,unused-private-member
                        self.__children.append(ch)
                else:
                    if isinstance(child, ModelElement):
                        child.__parent = self  # pylint: disable=protected-access,unused-private-member
                    self.__children.append(child)

    def find_children_of_type(self, typename):
        children_of_type = []
        for child in self.__children:
            if isinstance(child, typename):
                children_of_type.append(child)
            elif isinstance(child, ModelElement):
                children_of_type.extend(child.find_children_of_type(typename))
        return children_of_type

    def get_named_child(self, name, typename=None):
        # if typename is not none, type is checked
        for child in self.__children:
            if name == child.name:
                if typename:
                    if not isinstance(child, typename):
                        return None
                return child
        return None

    def find_first_child_of_type(self, typename, unique=True):
        found = None
        for child in self.__children:
            if isinstance(child, typename):
                if unique and found is not None:
                    raise ValueError(f"Child of type {typename} not unique.")
                found = child
        return found

    def get_child_with_expected_type(self, pos, typename):
        child = self.get_child(pos)
        if not isinstance(child, typename):
            raise OSC2ParsingError(
                msg=f'Child at pos {pos} is expected to be of type {typename.__name__}, but is {type(child).__name__}.', context=child.get_ctx())
        return child

    def find_parent(self, typename):
        if self.get_parent() is not None:
            child = self.get_parent().find_first_child_of_type(typename)
            if child:
                return child
            else:
                return self.get_parent().find_parent(typename)
        return None

    def find_reference_by_name(self, name, visited):

        for child in self.__children:
            if isinstance(child, ModelElement) and child not in visited:
                visited.append(child)
                if child.name == name:
                    return child

        if self.get_parent():
            found = self.get_parent().find_reference_by_name(name, visited)
            if found:
                return found
        return None

    def resolve(self, name):
        visited = [self]
        if self.get_parent() is not None:
            return self.get_parent().find_reference_by_name(name, visited)
        return None

    def get_parent(self):
        return self.__parent

    def delete_child(self, child):
        self.__children.remove(child)

    def has_siblings(self):
        if self.get_parent():
            return self.get_parent().get_child_count() > 1
        return False

    def set_loc(self, line, column):
        self.__line = line
        self.__column = column

    def set_ctx(self, ctx, filename: str):
        self.__line = ctx.start.line
        self.__column = ctx.start.column
        self.__context = ctx.getText()
        self.__filename = filename

    def get_ctx(self):
        return self.__line, self.__column, self.__context, self.__filename

    def accept(self, visitor):
        pass

    def enter_node(self, listener):
        pass

    def exit_node(self, listener):
        pass

    def __iter__(self):
        self.iter = iter(self.__children)
        return self.iter

    def __next__(self):
        return next(self.iter)

    def __str__(self) -> str:
        output = f"{self.__class__.__name__}({self.name})"
        first = True
        for attr in vars(self):
            if not attr.startswith('_') and attr != "name" and attr != "iter":
                if first:
                    output += ": "
                    first = False
                else:
                    output += ", "
                output += attr

                attr_val = getattr(self, attr)
                if isinstance(attr_val, ModelElement):
                    output += "=" + f"{attr_val.__class__.__name__}({attr_val.name})"
                else:
                    output += "=" + str(attr_val)
        return output


class CompilationUnit(ModelElement):

    def __init__(self):
        super().__init__()

    def enter_node(self, listener):
        if hasattr(listener, "enter_compilation_unit"):
            listener.enter_compilation_unit(self)

    def exit_node(self, listener):
        if hasattr(listener, "exit_compilation_unit"):
            listener.exit_compilation_unit(self)

    def accept(self, visitor):
        if hasattr(visitor, "visit_compilation_unit"):
            return visitor.visit_compilation_unit(self)
        else:
            return visitor.visit_children(self)


def merge_nested_dicts(dict1, dict2, key_must_exist=True):
    for key, value in dict2.items():
        if key_must_exist and key not in dict1:
            raise ValueError(f"Key not found {key}")
        if key in dict1 and isinstance(dict1[key], dict) and isinstance(value, dict):
            merge_nested_dicts(dict1[key], value, key_must_exist)
        else:
            dict1[key] = value


class Declaration(ModelElement):

    def __init__(self, name=""):
        super().__init__(name)
        self.base_type = None
        self.values = {}

    def get_declaration_string(self):
        return f"base_type: {self.base_type}, values: {self.values}"

    def get_qualified_name(self):
        res = ""
        current = self
        while current:
            if current.name:
                res = "/" + current.name + res
            current = current.get_parent()
        return res

    def get_base_type(self):
        return None

    def get_resolved_value(self, blackboard=None):
        return None

    def get_type_string(self):
        return self.name


class Parameter(Declaration):

    def get_value_child(self):
        if self.get_child_count() != 2:
            return None

        for child in self.get_children():
            if isinstance(child, (StringLiteral, FloatLiteral, BoolLiteral, IntegerLiteral, FunctionApplicationExpression, IdentifierReference, PhysicalLiteral, EnumValueReference, ListExpression, BinaryExpression, RelationExpression, LogicalExpression)):
                return child
            elif isinstance(child, KeepConstraintDeclaration):
                pass
            elif not isinstance(child, Type):
                raise OSC2ParsingError(msg=f'Parameter has invalid value "{type(child).__name__}".', context=self.get_ctx())
        return None

    def get_resolved_value(self, blackboard=None):
        param_type, is_list = self.get_type()
        vals = {}
        params = {}
        val_child = self.get_value_child()
        if val_child:
            vals = val_child.get_resolved_value(blackboard)

        if isinstance(param_type, StructuredDeclaration) and not is_list:
            params = param_type.get_resolved_value(blackboard)
            merge_nested_dicts(params, vals)
        else:
            params = vals

        for child in self.get_children():
            if isinstance(child, KeepConstraintDeclaration):  # for variable only?
                tmp = child.get_resolved_value(blackboard)
                merge_nested_dicts(params, tmp)
        return params

    def get_type(self):
        declared_type = self.find_first_child_of_type(Type)
        return declared_type.type_def, declared_type.is_list

    def get_type_string(self):
        val_type, is_list = self.get_type()

        if isinstance(val_type, ModelElement):
            val_type = val_type.get_type_string()
            if is_list:
                val_type = 'listof' + val_type
        return val_type


class StructuredDeclaration(Declaration):

    def get_parameter_names(self):
        names = []

        if self.get_base_type():
            names = self.get_base_type().get_parameter_names()

        for child in self.get_children():
            if isinstance(child, ParameterDeclaration):
                names.append(child.name)
        return list(set(names))

    def get_resolved_value(self, blackboard=None, skip_keys=None):
        if skip_keys is None:
            skip_keys = []
        params = {}

        # set values defined in base type
        if self.get_base_type():
            params = self.get_base_type().get_resolved_value(blackboard)

        named = False
        pos = 0
        param_keys = list(params.keys())
        for child in self.get_children():
            if isinstance(child, ParameterDeclaration):
                if child.name in skip_keys:
                    continue
                # set from parameter
                param_type, _ = child.get_type()

                # set values defined in type itself
                if isinstance(param_type, ModelElement):
                    params[child.name] = param_type.get_resolved_value(blackboard)

                # set values from parameter value
                val = child.get_value_child()
                if val:
                    if isinstance(val, KeepConstraintDeclaration):
                        tmp = val.get_resolved_value(blackboard)
                        for key, val in tmp.items():
                            if key not in params:
                                raise OSC2ParsingError(
                                    msg=f'Keep Constraint Declaration specifies unknown member "{key}".', context=self.get_ctx())
                            else:
                                params[key] = val
                    else:
                        params[child.name] = val.get_resolved_value(blackboard)
                else:
                    if child.name not in params:
                        params[child.name] = None
            elif isinstance(child, PositionalArgument):
                if named:
                    raise OSC2ParsingError(
                        msg=f'Positional argument after named argument not allowed.', context=child.get_ctx())
                if param_keys[pos] not in skip_keys:
                    params[param_keys[pos]] = child.get_resolved_value(blackboard)
                pos += 1
            elif isinstance(child, NamedArgument):
                named = True
                if child.name not in skip_keys:
                    params[child.name] = child.get_resolved_value(blackboard)
            elif isinstance(child, KeepConstraintDeclaration):
                tmp = child.get_resolved_value(blackboard)
                merge_nested_dicts(params, tmp, key_must_exist=False)
            elif isinstance(child, MethodDeclaration):
                if child.name not in skip_keys:
                    params[child.name] = child.get_resolved_value(blackboard)

        return params

    def get_type(self):
        return self, False

    def get_type_string(self):
        return self.name


class ModelExpression(ModelElement):
    pass


class PhysicalTypeDeclaration(Declaration):

    def __init__(self, type_name):
        super().__init__(type_name)
        self.type_name = type_name

    def enter_node(self, listener):
        if hasattr(listener, "enter_physical_type_declaration"):
            listener.enter_physical_type_declaration(self)

    def exit_node(self, listener):
        if hasattr(listener, "exit_physical_type_declaration"):
            listener.exit_physical_type_declaration(self)

    def accept(self, visitor):
        if hasattr(visitor, "visit_physical_type_declaration"):
            return visitor.visit_physical_type_declaration(self)
        else:
            return visitor.visit_children(self)

    def get_resolved_value(self, blackboard=None):
        return None

    def get_type_string(self):
        return self.type_name


class UnitDeclaration(Declaration):

    def __init__(self, unit_name, physical_name):
        super().__init__(unit_name)
        self.unit_name = unit_name
        self.physical_name = physical_name

    def enter_node(self, listener):
        if hasattr(listener, "enter_unit_declaration"):
            listener.enter_unit_declaration(self)

    def exit_node(self, listener):
        if hasattr(listener, "exit_unit_declaration"):
            listener.exit_unit_declaration(self)

    def accept(self, visitor):
        if hasattr(visitor, "visit_unit_declaration"):
            return visitor.visit_unit_declaration(self)
        else:
            return visitor.visit_children(self)

    def get_type_string(self):
        return self.physical_name.name


class SIUnitSpecifier(ModelElement):

    def __init__(self, factor, offset):
        super().__init__()
        self.factor = factor
        self.offset = offset

    def enter_node(self, listener):
        if hasattr(listener, "enter_si_unit_specifier"):
            listener.enter_si_unit_specifier(self)

    def exit_node(self, listener):
        if hasattr(listener, "exit_si_unit_specifier"):
            listener.exit_si_unit_specifier(self)

    def accept(self, visitor):
        if hasattr(visitor, "visit_si_unit_specifier"):
            return visitor.visit_si_unit_specifier(self)
        else:
            return visitor.visit_children(self)


class SIBaseExponent(ModelElement):

    def __init__(self, unit_name):
        super().__init__()
        self.unit_name = unit_name

    def enter_node(self, listener):
        if hasattr(listener, "enter_si_base_exponent"):
            listener.enter_si_base_exponent(self)

    def exit_node(self, listener):
        if hasattr(listener, "exit_si_base_exponent"):
            listener.exit_si_base_exponent(self)

    def accept(self, visitor):
        if hasattr(visitor, "visit_si_base_exponent"):
            return visitor.visit_si_base_exponent(self)
        else:
            return visitor.visit_children(self)


class EnumDeclaration(Declaration):

    def __init__(self, enum_name):
        super().__init__(enum_name)
        self.enum_name = enum_name

    def enter_node(self, listener):
        if hasattr(listener, "enter_enum_declaration"):
            listener.enter_enum_declaration(self)

    def exit_node(self, listener):
        if hasattr(listener, "exit_enum_declaration"):
            listener.exit_enum_declaration(self)

    def accept(self, visitor):
        if hasattr(visitor, "visit_enum_declaration"):
            return visitor.visit_enum_declaration(self)
        else:
            return visitor.visit_children(self)

    def get_resolved_value(self, blackboard=None):
        return None


class EnumMemberDeclaration(Declaration):

    def __init__(self, member_name, numeric_value):
        super().__init__()
        self.member_name = member_name
        self.numeric_value = numeric_value

    def enter_node(self, listener):
        if hasattr(listener, "enter_enum_member_decl"):
            listener.enter_enum_member_decl(self)

    def exit_node(self, listener):
        if hasattr(listener, "exit_enum_member_decl"):
            listener.exit_enum_member_decl(self)

    def accept(self, visitor):
        if hasattr(visitor, "visit_enum_member_decl"):
            return visitor.visit_enum_member_declaration(self)
        else:
            return visitor.visit_children(self)


class EnumValueReference(ModelElement):

    def __init__(self, enum_name, enum_member_name):
        super().__init__()
        self.enum_name = enum_name
        self.enum_member_name = enum_member_name

    def enter_node(self, listener):
        if hasattr(listener, "enter_enum_value_reference"):
            listener.enter_enum_value_reference(self)

    def exit_node(self, listener):
        if hasattr(listener, "exit_enum_value_reference"):
            listener.exit_enum_value_reference(self)

    def accept(self, visitor):
        if hasattr(visitor, "visit_enum_value_reference"):
            return visitor.visit_enum_value_reference(self)
        else:
            return visitor.visit_children(self)

    def get_type_string(self):
        return self.enum_name.name

    def get_resolved_value(self, blackboard=None):
        return (self.enum_member_name.member_name, self.enum_member_name.numeric_value)


class InheritsCondition(ModelElement):

    def __init__(self, field_name, bool_literal):
        super().__init__()
        self.field_name = field_name

    def enter_node(self, listener):
        if hasattr(listener, "enter_inherits_condition"):
            listener.enter_inherits_condition(self)

    def exit_node(self, listener):
        if hasattr(listener, "exit_inherits_condition"):
            listener.exit_inherits_condition(self)

    def accept(self, visitor):
        if hasattr(visitor, "visit_inherits_condition"):
            return visitor.visit_inherits_condition(self)
        else:
            return visitor.visit_children(self)


class StructDeclaration(StructuredDeclaration):

    def __init__(self, struct_name):
        super().__init__(struct_name)
        self.struct_name = struct_name

    def enter_node(self, listener):
        if hasattr(listener, "enter_struct_declaration"):
            listener.enter_struct_declaration(self)

    def exit_node(self, listener):
        if hasattr(listener, "exit_struct_declaration"):
            listener.exit_struct_declaration(self)

    def accept(self, visitor):
        if hasattr(visitor, "visit_struct_declaration"):
            return visitor.visit_struct_declaration(self)
        else:
            return visitor.visit_children(self)

    def get_base_type(self):
        inherits = self.find_first_child_of_type(StructInherits)
        if inherits:
            return inherits.struct_name
        return None


class Inheritance(ModelElement):

    def __init__(self):
        super().__init__()


class StructInherits(Inheritance):

    def __init__(self, struct_name):
        super().__init__()
        self.struct_name = struct_name

    def enter_node(self, listener):
        if hasattr(listener, "enter_struct_inherits"):
            listener.enter_struct_inherits(self)

    def exit_node(self, listener):
        if hasattr(listener, "exit_struct_inherits"):
            listener.exit_struct_inherits(self)

    def accept(self, visitor):
        if hasattr(visitor, "visit_struct_inherits"):
            return visitor.visit_struct_inherits(self)
        else:
            return visitor.visit_children(self)


class ActorDeclaration(StructuredDeclaration):

    def __init__(self, actor):
        super().__init__(actor)
        self.actor = actor

    def enter_node(self, listener):
        if hasattr(listener, "enter_actor_declaration"):
            listener.enter_actor_declaration(self)

    def exit_node(self, listener):
        if hasattr(listener, "exit_actor_declaration"):
            listener.exit_actor_declaration(self)

    def accept(self, visitor):
        if hasattr(visitor, "visit_actor_declaration"):
            return visitor.visit_actor_declaration(self)
        else:
            return visitor.visit_children(self)

    def get_base_type(self):
        inherits = self.find_first_child_of_type(ActorInherits)
        if inherits:
            return inherits.actor
        return None


class ActorInherits(Inheritance):

    def __init__(self, actor):
        super().__init__()
        self.actor = actor

    def enter_node(self, listener):
        if hasattr(listener, "enter_actor_inherits"):
            listener.enter_actor_inherits(self)

    def exit_node(self, listener):
        if hasattr(listener, "exit_actor_inherits"):
            listener.exit_actor_inherits(self)

    def accept(self, visitor):
        if hasattr(visitor, "visit_actor_inherits"):
            return visitor.visit_actor_inherits(self)
        else:
            return visitor.visit_children(self)


class ScenarioDeclaration(StructuredDeclaration):

    def __init__(self, qualified_behavior_name):
        super().__init__(qualified_behavior_name)
        self.qualified_behavior_name = qualified_behavior_name

    def enter_node(self, listener):
        if hasattr(listener, "enter_scenario_declaration"):
            listener.enter_scenario_declaration(self)

    def exit_node(self, listener):
        if hasattr(listener, "exit_scenario_declaration"):
            listener.exit_scenario_declaration(self)

    def accept(self, visitor):
        if hasattr(visitor, "visit_scenario_declaration"):
            return visitor.visit_scenario_declaration(self)
        else:
            return visitor.visit_children(self)

    def get_base_type(self):
        inherits = self.find_first_child_of_type(ScenarioInherits)
        if inherits:
            return inherits.qualified_behavior_name
        return None


class ScenarioInherits(Inheritance):

    def __init__(self, qualified_behavior_name):
        super().__init__()
        self.qualified_behavior_name = qualified_behavior_name

    def enter_node(self, listener):
        if hasattr(listener, "enter_scenario_inherits"):
            listener.enter_scenario_inherits(self)

    def exit_node(self, listener):
        if hasattr(listener, "exit_scenario_inherits"):
            listener.exit_scenario_inherits(self)

    def accept(self, visitor):
        if hasattr(visitor, "visit_scenario_inherits"):
            return visitor.visit_scenario_inherits(self)
        else:
            return visitor.visit_children(self)


class ActionDeclaration(StructuredDeclaration):

    def __init__(self, qualified_behavior_name):
        self.actor = None
        super().__init__(qualified_behavior_name)
        self.qualified_behavior_name = qualified_behavior_name

    def enter_node(self, listener):
        if hasattr(listener, "enter_action_declaration"):
            listener.enter_action_declaration(self)

    def exit_node(self, listener):
        if hasattr(listener, "exit_action_declaration"):
            listener.exit_action_declaration(self)

    def accept(self, visitor):
        if hasattr(visitor, "visit_action_declaration"):
            return visitor.visit_action_declaration(self)
        else:
            return visitor.visit_children(self)

    def get_base_type(self):
        inherits = self.find_first_child_of_type(ActionInherits)
        if inherits:
            return inherits.qualified_behavior_name
        return None


class ActionInherits(Inheritance):

    def __init__(self, qualified_behavior_name):
        super().__init__()
        self.qualified_behavior_name = qualified_behavior_name

    def enter_node(self, listener):
        if hasattr(listener, "enter_action_inherits"):
            listener.enter_action_inherits(self)

    def exit_node(self, listener):
        if hasattr(listener, "exit_action_inherits"):
            listener.exit_action_inherits(self)

    def accept(self, visitor):
        if hasattr(visitor, "visit_action_inherits"):
            return visitor.visit_action_inherits(self)
        else:
            return visitor.visit_children(self)


class ModifierDeclaration(StructuredDeclaration):

    def __init__(self, actor, name):
        if actor:
            super().__init__(actor + "." + name)
        else:
            super().__init__(name)
        self.actor = actor
        self.modifier = name

    def enter_node(self, listener):
        if hasattr(listener, "enter_modifier_declaration"):
            listener.enter_modifier_declaration(self)

    def exit_node(self, listener):
        if hasattr(listener, "exit_modifier_declaration"):
            listener.exit_modifier_declaration(self)

    def accept(self, visitor):
        if hasattr(visitor, "visit_modifier_declaration"):
            return visitor.visit_modifier_declaration(self)
        else:
            return visitor.visit_children(self)


class EnumTypeExtension(Declaration):

    def __init__(self, enum_name):
        super().__init__()
        self.enum_name = enum_name

    def enter_node(self, listener):
        if hasattr(listener, "enter_enum_type_extension"):
            listener.enter_enum_type_extension(self)

    def exit_node(self, listener):
        if hasattr(listener, "exit_enum_type_extension"):
            listener.exit_enum_type_extension(self)

    def accept(self, visitor):
        if hasattr(visitor, "visit_enum_type_extension"):
            return visitor.visit_enum_type_extension(self)
        else:
            return visitor.visit_children(self)


class StructuredTypeExtension(Declaration):

    def __init__(self, type_name, qualified_behavior_name):
        super().__init__()
        self.type_name = type_name
        self.qualified_behavior_name = qualified_behavior_name

    def enter_node(self, listener):
        if hasattr(listener, "enter_structured_type_extension"):
            listener.enter_structured_type_extension(self)

    def exit_node(self, listener):
        if hasattr(listener, "exit_structured_type_extension"):
            listener.exit_structured_type_extension(self)

    def accept(self, visitor):
        if hasattr(visitor, "visit_structured_type_extension"):
            return visitor.visit_structured_type_extension(self)
        else:
            return visitor.visit_children(self)


class GlobalParameterDeclaration(Parameter):

    def __init__(self, name, field_name, field_type, default_value):
        super().__init__(name)
        self.field_name = field_name
        self.field_type = field_type
        self.default_value = default_value

    def enter_node(self, listener):
        if hasattr(listener, "enter_global_parameter_declaration"):
            listener.enter_global_parameter_declaration(self)

    def exit_node(self, listener):
        if hasattr(listener, "exit_global_parameter_declaration"):
            listener.exit_global_parameter_declaration(self)

    def accept(self, visitor):
        if hasattr(visitor, "visit_global_parameter_declaration"):
            return visitor.visit_global_parameter_declaration(self)
        else:
            return visitor.visit_children(self)


class ParameterDeclaration(Parameter):

    def __init__(self, field_name, field_type, default_value):
        super().__init__(field_name)
        self.field_name = field_name
        self.field_type = field_type
        self.default_value = default_value

    def enter_node(self, listener):
        if hasattr(listener, "enter_parameter_declaration"):
            listener.enter_parameter_declaration(self)

    def exit_node(self, listener):
        if hasattr(listener, "exit_parameter_declaration"):
            listener.exit_parameter_declaration(self)

    def accept(self, visitor):
        if hasattr(visitor, "visit_parameter_declaration"):
            return visitor.visit_parameter_declaration(self)
        else:
            return visitor.visit_children(self)

    def get_fully_qualified_var_name(self, include_scenario):
        name = self.name
        parent = self.get_parent()
        while parent and not isinstance(parent, ScenarioDeclaration):
            name = parent.name + "/" + name
            parent = parent.get_parent()
        if include_scenario and parent and parent.name:
            name = parent.name + "/" + name
        return name


class ParameterReference(ModelElement):

    def __init__(self, field_name, field_access):
        super().__init__()
        self.field_name = field_name
        self.field_access = field_access

    def enter_node(self, listener):
        if hasattr(listener, "enter_parameter_reference"):
            listener.enter_parameter_reference(self)

    def exit_node(self, listener):
        if hasattr(listener, "exit_parameter_reference"):
            listener.exit_parameter_reference(self)

    def accept(self, visitor):
        if hasattr(visitor, "visit_parameter_reference"):
            return visitor.visit_parameter_reference(self)
        else:
            return visitor.visit_children(self)


class EventDeclaration(Declaration):

    def __init__(self, event_name):
        super().__init__(event_name)
        self.field_name = event_name

    def enter_node(self, listener):
        if hasattr(listener, "enter_event_declaration"):
            listener.enter_event_declaration(self)

    def exit_node(self, listener):
        if hasattr(listener, "exit_event_declaration"):
            listener.exit_event_declaration(self)

    def accept(self, visitor):
        if hasattr(visitor, "visit_event_declaration"):
            return visitor.visit_event_declaration(self)
        else:
            return visitor.visit_children(self)


class EventReference(ModelElement):

    def __init__(self, event_path):
        super().__init__()
        self.event_path = event_path

    def enter_node(self, listener):
        if hasattr(listener, "enter_event_reference"):
            listener.enter_event_reference(self)

    def exit_node(self, listener):
        if hasattr(listener, "exit_event_reference"):
            listener.exit_event_reference(self)

    def accept(self, visitor):
        if hasattr(visitor, "visit_event_reference"):
            return visitor.visit_event_reference(self)
        else:
            return visitor.visit_children(self)


class EventFieldDecl(ModelElement):

    def __init__(self, event_field_name):
        super().__init__()
        self.event_field_name = event_field_name

    def enter_node(self, listener):
        if hasattr(listener, "enter_event_field_declaration"):
            listener.enter_event_field_declaration(self)

    def exit_node(self, listener):
        if hasattr(listener, "exit_event_field_declaration"):
            listener.exit_event_field_declaration(self)

    def accept(self, visitor):
        if hasattr(visitor, "visit_event_field_declaration"):
            return visitor.visit_event_field_declaration(self)
        else:
            return visitor.visit_children(self)


class EventCondition(ModelElement):

    def __init__(self):
        super().__init__()

    def enter_node(self, listener):
        if hasattr(listener, "enter_event_condition"):
            listener.enter_event_condition(self)

    def exit_node(self, listener):
        if hasattr(listener, "exit_event_condition"):
            listener.exit_event_condition(self)

    def accept(self, visitor):
        if hasattr(visitor, "visit_event_condition"):
            return visitor.visit_event_condition(self)
        else:
            return visitor.visit_children(self)


class MethodDeclaration(Declaration):

    def __init__(self, method_name, return_type):
        super().__init__(method_name)
        self.method_name = method_name
        self.return_type = return_type

    def enter_node(self, listener):
        if hasattr(listener, "enter_method_declaration"):
            listener.enter_method_declaration(self)

    def exit_node(self, listener):
        if hasattr(listener, "exit_method_declaration"):
            listener.exit_method_declaration(self)

    def accept(self, visitor):
        if hasattr(visitor, "visit_method_declaration"):
            return visitor.visit_method_declaration(self)
        else:
            return visitor.visit_children(self)

    def get_resolved_value(self, blackboard=None):
        params = {}
        for child in self.get_children():
            if isinstance(child, Argument):
                params[child.name] = child.default_value
        return params


class MethodBody(ModelElement):

    def __init__(self, qualifier, type_ref, external_name):
        super().__init__()
        self.qualifier = qualifier
        self.type_ref = type_ref
        self.external_name = external_name

    def enter_node(self, listener):
        if hasattr(listener, "enter_method_body"):
            listener.enter_method_body(self)

    def exit_node(self, listener):
        if hasattr(listener, "exit_method_body"):
            listener.exit_method_body(self)

    def accept(self, visitor):
        if hasattr(visitor, "visit_method_body"):
            return visitor.visit_method_body(self)
        else:
            return visitor.visit_children(self)


class CoverDeclaration(Declaration):

    def __init__(self, target_name):
        super().__init__()
        self.target_name = target_name

    def enter_node(self, listener):
        if hasattr(listener, "enter_cover_declaration"):
            listener.enter_cover_declaration(self)

    def exit_node(self, listener):
        if hasattr(listener, "exit_cover_declaration"):
            listener.exit_cover_declaration(self)

    def accept(self, visitor):
        if hasattr(visitor, "visit_cover_declaration"):
            return visitor.visit_cover_declaration(self)
        else:
            return visitor.visit_children(self)


class RecordDeclaration(Declaration):

    def __init__(self, target_name):
        super().__init__()
        self.target_name = target_name

    def enter_node(self, listener):
        if hasattr(listener, "enter_record_declaration"):
            listener.enter_record_declaration(self)

    def exit_node(self, listener):
        if hasattr(listener, "exit_record_declaration"):
            listener.exit_record_declaration(self)

    def accept(self, visitor):
        if hasattr(visitor, "visit_record_declaration"):
            return visitor.visit_record_declaration(self)
        else:
            return visitor.visit_children(self)


class Argument(Parameter):

    def __init__(self, argument_name, argument_type, default_value):
        super().__init__(argument_name)
        self.argument_name = argument_name
        self.argument_type = argument_type
        self.default_value = default_value

    def enter_node(self, listener):
        if hasattr(listener, "enter_argument"):
            listener.enter_argument(self)

    def exit_node(self, listener):
        if hasattr(listener, "exit_argument"):
            listener.exit_argument(self)

    def accept(self, visitor):
        if hasattr(visitor, "visit_argument"):
            return visitor.visit_argument(self)
        else:
            return visitor.visit_children(self)


class NamedArgument(ModelElement):

    def __init__(self, argument_name):
        super().__init__(argument_name)
        self.argument_name = argument_name

    def enter_node(self, listener):
        if hasattr(listener, "enter_named_argument"):
            listener.enter_named_argument(self)

    def exit_node(self, listener):
        if hasattr(listener, "exit_named_argument"):
            listener.exit_named_argument(self)

    def accept(self, visitor):
        if hasattr(visitor, "visit_named_argument"):
            return visitor.visit_named_argument(self)
        else:
            return visitor.visit_children(self)

    def get_resolved_value(self, blackboard=None):
        if self.get_child_count() != 1:
            raise OSC2ParsingError(
                msg=f'Could not get value of positional argument because the expected child count is not 1, but {self.get_child_count()}.', context=self.get_ctx())
        return self.get_child(0).get_resolved_value(blackboard)


class PositionalArgument(ModelElement):

    def __init__(self):
        super().__init__()

    def enter_node(self, listener):
        if hasattr(listener, "enter_positional_argument"):
            listener.enter_positional_argument(self)

    def exit_node(self, listener):
        if hasattr(listener, "exit_positional_argument"):
            listener.exit_positional_argument(self)

    def accept(self, visitor):
        if hasattr(visitor, "visit_positional_argument"):
            return visitor.visit_positional_argument(self)
        else:
            return visitor.visit_children(self)

    def get_resolved_value(self, blackboard=None):
        if self.get_child_count() != 1:
            raise OSC2ParsingError(
                msg=f'Could not get value of positional argument because the expected child count is not 1, but {self.get_child_count()}.', context=self.get_ctx())
        return self.get_child(0).get_resolved_value(blackboard)


class VariableDeclaration(Parameter):

    def __init__(self, field_name, field_type, default_value):
        super().__init__(field_name)
        self.field_name = field_name  # unused?
        self.field_type = field_type
        self.default_value = default_value
        # self.set_children(field_name)

    def enter_node(self, listener):
        if hasattr(listener, "enter_variable_declaration"):
            listener.enter_variable_declaration(self)

    def exit_node(self, listener):
        if hasattr(listener, "exit_variable_declaration"):
            listener.exit_variable_declaration(self)

    def accept(self, visitor):
        if hasattr(visitor, "visit_variable_declaration"):
            return visitor.visit_variable_declaration(self)
        else:
            return visitor.visit_children(self)


class KeepConstraintDeclaration(Declaration):

    def __init__(self, constraint_qualifier):
        super().__init__()
        self.constraint_qualifier = constraint_qualifier

    def enter_node(self, listener):
        if hasattr(listener, "enter_keep_constraint_declaration"):
            listener.enter_keep_constraint_declaration(self)

    def exit_node(self, listener):
        if hasattr(listener, "exit_keep_constraint_declaration"):
            listener.exit_keep_constraint_declaration(self)

    def accept(self, visitor):
        if hasattr(visitor, "visit_keep_constraint_declaration"):
            return visitor.visit_keep_constraint_declaration(self)
        else:
            return visitor.visit_children(self)

    def get_resolved_value(self, blackboard=None):
        result = None
        if self.get_child_count() == 1 and isinstance(self.get_child(0), RelationExpression) and self.get_child(0).get_child_count() == 2:
            if self.get_child(0).operator != "==":
                raise OSC2ParsingError(
                    msg=f'Only relation "==" is currently supported in "keep".', context=self.get_ctx())
            field_exp = self.get_child(0).get_child_with_expected_type(0, FieldAccessExpression)
            value = self.get_child(0).get_child(1).get_resolved_value(blackboard)

            if not field_exp.field_name.startswith('it.'):
                raise OSC2ParsingError(
                    msg=f'FieldAccessExpression only supports "it." prefix, not "{field_exp.field_name}".', context=self.get_ctx())
            field_name = field_exp.field_name.removeprefix("it.")
            result = {}
            member_path = field_name.split('.')
            current_params = result
            for current_pos in range(0, len(member_path)):  # pylint: disable=consider-using-enumerate
                if current_pos != len(member_path) - 1:
                    current_params[member_path[current_pos]] = {}
                    current_params = current_params[member_path[current_pos]]
                else:
                    current_params[member_path[current_pos]] = value
        else:
            raise OSC2ParsingError(
                msg=f'Keep uses unsupported expression: allowed "==" only.', context=self.get_ctx())

        if result is None:
            raise OSC2ParsingError(
                msg=f'Error in keep constraint declaration.', context=self.get_ctx())

        return result


class RemoveDefaultDeclaration(Declaration):

    def __init__(self):
        super().__init__()

    def enter_node(self, listener):
        if hasattr(listener, "enter_remove_default_declaration"):
            listener.enter_remove_default_declaration(self)

    def exit_node(self, listener):
        if hasattr(listener, "exit_remove_default_declaration"):
            listener.exit_remove_default_declaration(self)

    def accept(self, visitor):
        if hasattr(visitor, "visit_remove_default_declaration"):
            return visitor.visit_remove_default_declaration(self)
        else:
            return visitor.visit_children(self)


class OnDirective(ModelElement):

    def __init__(self):
        super().__init__()

    def enter_node(self, listener):
        if hasattr(listener, "enter_on_directive"):
            listener.enter_on_directive(self)

    def exit_node(self, listener):
        if hasattr(listener, "exit_on_directive"):
            listener.exit_on_directive(self)

    def accept(self, visitor):
        if hasattr(visitor, "visit_on_directive"):
            return visitor.visit_on_directive(self)
        else:
            return visitor.visit_children(self)


class DoDirective(ModelElement):

    def __init__(self):
        super().__init__()

    def enter_node(self, listener):
        if hasattr(listener, "enter_do_directive"):
            listener.enter_do_directive(self)

    def exit_node(self, listener):
        if hasattr(listener, "exit_do_directive"):
            listener.exit_do_directive(self)

    def accept(self, visitor):
        if hasattr(visitor, "visit_do_directive"):
            return visitor.visit_do_directive(self)
        else:
            return visitor.visit_children(self)


class DoMember(ModelElement):

    def __init__(self, label_name, composition_operator):
        super().__init__(label_name)
        self.label_name = label_name
        self.composition_operator = composition_operator

    def enter_node(self, listener):
        if hasattr(listener, "enter_do_member"):
            listener.enter_do_member(self)

    def exit_node(self, listener):
        if hasattr(listener, "exit_do_member"):
            listener.exit_do_member(self)

    def accept(self, visitor):
        if hasattr(visitor, "visit_do_member"):
            return visitor.visit_do_member(self)
        else:
            return visitor.visit_children(self)


class WaitDirective(ModelElement):

    def __init__(self):
        super().__init__()

    def enter_node(self, listener):
        if hasattr(listener, "enter_wait_directive"):
            listener.enter_wait_directive(self)

    def exit_node(self, listener):
        if hasattr(listener, "exit_wait_directive"):
            listener.exit_wait_directive(self)

    def accept(self, visitor):
        if hasattr(visitor, "visit_wait_directive"):
            return visitor.visit_wait_directive(self)
        else:
            return visitor.visit_children(self)


class EmitDirective(ModelElement):

    def __init__(self, event_name):
        super().__init__(event_name)
        self.event_name = event_name

    def enter_node(self, listener):
        if hasattr(listener, "enter_emit_directive"):
            listener.enter_emit_directive(self)

    def exit_node(self, listener):
        if hasattr(listener, "exit_emit_directive"):
            listener.exit_emit_directive(self)

    def accept(self, visitor):
        if hasattr(visitor, "visit_emit_directive"):
            return visitor.visit_emit_directive(self)
        else:
            return visitor.visit_children(self)


class CallDirective(ModelElement):

    def __init__(self, method_name):
        super().__init__()
        self.method_name = method_name

    def enter_node(self, listener):
        if hasattr(listener, "enter_call_directive"):
            listener.enter_call_directive(self)

    def exit_node(self, listener):
        if hasattr(listener, "exit_call_directive"):
            listener.exit_call_directive(self)

    def accept(self, visitor):
        if hasattr(visitor, "visit_call_directive"):
            return visitor.visit_call_directive(self)
        else:
            return visitor.visit_children(self)


class UntilDirective(ModelElement):

    def __init__(self):
        super().__init__()

    def enter_node(self, listener):
        if hasattr(listener, "enter_until_directive"):
            listener.enter_until_directive(self)

    def exit_node(self, listener):
        if hasattr(listener, "exit_until_directive"):
            listener.exit_until_directive(self)

    def accept(self, visitor):
        if hasattr(visitor, "visit_until_directive"):
            return visitor.visit_until_directive(self)
        else:
            return visitor.visit_children(self)


class BehaviorInvocation(StructuredDeclaration):

    def __init__(self, name, actor, behavior):
        super().__init__(name)
        self.actor = actor
        self.behavior = behavior

    def enter_node(self, listener):
        if hasattr(listener, "enter_behavior_invocation"):
            listener.enter_behavior_invocation(self)

    def exit_node(self, listener):
        if hasattr(listener, "exit_behavior_invocation"):
            listener.exit_behavior_invocation(self)

    def accept(self, visitor):
        if hasattr(visitor, "visit_behavior_invocation"):
            return visitor.visit_behavior_invocation(self)
        else:
            return visitor.visit_children(self)

    def get_base_type(self):
        return self.behavior

    def get_type(self):
        return self.behavior, False

    def get_resolved_value_with_variable_references(self, blackboard):
        params = self.get_resolved_value(blackboard)

        pos = 0
        param_keys = list(params.keys())
        for child in self.get_children():
            if isinstance(child, PositionalArgument):
                if isinstance(child.get_child(0), IdentifierReference):
                    params[param_keys[pos]] = child.get_child(0).get_blackboard_reference(blackboard)
                pos += 1
            elif isinstance(child, NamedArgument):
                if isinstance(child.get_child(0), IdentifierReference):
                    params[child.name] = child.get_child(0).get_blackboard_reference(blackboard)

        return params


class ModifierInvocation(StructuredDeclaration):

    def __init__(self, actor, modifier):
        super().__init__()
        self.actor = actor
        self.modifier = modifier

    def enter_node(self, listener):
        if hasattr(listener, "enter_modifier_invocation"):
            listener.enter_modifier_invocation(self)

    def exit_node(self, listener):
        if hasattr(listener, "exit_modifier_invocation"):
            listener.exit_modifier_invocation(self)

    def accept(self, visitor):
        if hasattr(visitor, "visit_modifier_invocation"):
            return visitor.visit_modifier_invocation(self)
        else:
            return visitor.visit_children(self)

    def get_base_type(self):
        return self.modifier


class RiseExpression(ModelExpression):

    def __init__(self):
        super().__init__()

    def enter_node(self, listener):
        if hasattr(listener, "enter_rise_expression"):
            listener.enter_rise_expression(self)

    def exit_node(self, listener):
        if hasattr(listener, "exit_rise_expression"):
            listener.exit_rise_expression(self)

    def accept(self, visitor):
        if hasattr(visitor, "visit_rise_expression"):
            return visitor.visit_rise_expression(self)
        else:
            return visitor.visit_children(self)


class FallExpression(ModelExpression):

    def __init__(self):
        super().__init__()

    def enter_node(self, listener):
        if hasattr(listener, "enter_fall_expression"):
            listener.enter_rise_expression(self)

    def exit_node(self, listener):
        if hasattr(listener, "exit_fall_expression"):
            listener.exit_fall_expression(self)

    def accept(self, visitor):
        if hasattr(visitor, "visit_fall_expression"):
            return visitor.visit_fall_expression(self)
        else:
            return visitor.visit_children(self)


class ElapsedExpression(ModelExpression):

    def __init__(self):
        super().__init__()

    def enter_node(self, listener):
        if hasattr(listener, "enter_elapsed_expression"):
            listener.enter_rise_expression(self)

    def exit_node(self, listener):
        if hasattr(listener, "exit_elapsed_expression"):
            listener.exit_fall_expression(self)

    def accept(self, visitor):
        if hasattr(visitor, "visit_elapsed_expression"):
            return visitor.visit_fall_expression(self)
        else:
            return visitor.visit_children(self)


class EveryExpression(ModelExpression):

    def __init__(self):
        super().__init__()

    def enter_node(self, listener):
        if hasattr(listener, "enter_every_expression"):
            listener.enter_every_expression(self)

    def exit_node(self, listener):
        if hasattr(listener, "exit_every_expression"):
            listener.exit_every_expression(self)

    def accept(self, visitor):
        if hasattr(visitor, "visit_every_expression"):
            return visitor.visit_every_expression(self)
        else:
            return visitor.visit_children(self)


class SampleExpression(ModelExpression):

    def __init__(self):
        super().__init__()

    def enter_node(self, listener):
        if hasattr(listener, "enter_sample_expression"):
            listener.enter_sample_expression(self)

    def exit_node(self, listener):
        if hasattr(listener, "exit_sample_expression"):
            listener.exit_sample_expression(self)

    def accept(self, visitor):
        if hasattr(visitor, "visit_sample_expression"):
            return visitor.visit_sample_expression(self)
        else:
            return visitor.visit_children(self)


class CastExpression(ModelExpression):

    def __init__(self, object_def, target_type):
        super().__init__()
        self.object_def = object_def
        self.target_type = target_type

    def enter_node(self, listener):
        if hasattr(listener, "enter_cast_expression"):
            listener.enter_cast_expression(self)

    def exit_node(self, listener):
        if hasattr(listener, "exit_cast_expression"):
            listener.exit_cast_expression(self)

    def accept(self, visitor):
        if hasattr(visitor, "visit_cast_expression"):
            return visitor.visit_cast_expression(self)
        else:
            return visitor.visit_children(self)


class TypeTestExpression(ModelExpression):

    def __init__(self, object_def, target_type):
        super().__init__()
        self.object_def = object_def
        self.target_type = target_type

    def enter_node(self, listener):
        if hasattr(listener, "enter_type_test_expression"):
            listener.enter_type_test_expression(self)

    def exit_node(self, listener):
        if hasattr(listener, "exit_type_test_expression"):
            listener.exit_type_test_expression(self)

    def accept(self, visitor):
        if hasattr(visitor, "visit_type_test_expression"):
            return visitor.visit_type_test_expression(self)
        else:
            return visitor.visit_children(self)


class ElementAccessExpression(ModelExpression):

    def __init__(self, list_name, index):
        super().__init__()
        self.list_name = list_name
        self.index = index

    def enter_node(self, listener):
        if hasattr(listener, "enter_element_access_expression"):
            listener.enter_element_access_expression(self)

    def exit_node(self, listener):
        if hasattr(listener, "exit_element_access_expression"):
            listener.exit_element_access_expression(self)

    def accept(self, visitor):
        if hasattr(visitor, "visit_element_access_expression"):
            return visitor.visit_element_access_expression(self)
        else:
            return visitor.visit_children(self)


class FunctionApplicationExpression(ModelExpression):

    def __init__(self, func_name):
        super().__init__()
        self.func_name = func_name

    def enter_node(self, listener):
        if hasattr(listener, "enter_function_application_expression"):
            listener.enter_function_application_expression(self)

    def exit_node(self, listener):
        if hasattr(listener, "exit_function_application_expression"):
            listener.exit_function_application_expression(self)

    def accept(self, visitor):
        if hasattr(visitor, "visit_function_application_expression"):
            return visitor.visit_function_application_expression(self)
        else:
            return visitor.visit_children(self)

    def get_resolved_value(self, blackboard=None):
        ref = self.find_first_child_of_type(IdentifierReference)
        params = ref.get_resolved_value(blackboard)
        named = False
        pos = 0
        param_keys = list(params.keys())
        for child in self.get_children():
            key = None
            if isinstance(child, PositionalArgument):
                if named:
                    raise OSC2ParsingError(
                        msg=f'Positional argument after named argument not allowed.', context=child.get_ctx())
                if len(param_keys) <= pos:
                    raise OSC2ParsingError(
                        msg=f'Positional argument at position {pos} not expected.', context=child.get_ctx())
                key = param_keys[pos]
                pos += 1
            elif isinstance(child, NamedArgument):
                named = True
                key = child.name
            if key:
                params[key] = child.get_resolved_value(blackboard)

        if isinstance(ref.ref, list) and isinstance(ref.ref[-1], MethodDeclaration):
            body = ref.ref[-1].find_first_child_of_type(MethodBody)
            try:
                params = body.external_name(**params)
            except Exception as e:
                raise ValueError(f'Error while calling external method: {e}') from e

        return params

    def get_type(self):
        ref = self.find_first_child_of_type(IdentifierReference)
        return ref.get_type()

    def get_type_string(self):
        return self.get_type()[0].name


class FieldAccessExpression(ModelExpression):

    def __init__(self, field_name):
        super().__init__()
        self.field_name = field_name

    def enter_node(self, listener):
        if hasattr(listener, "enter_field_access_expression"):
            listener.enter_field_access_expression(self)

    def exit_node(self, listener):
        if hasattr(listener, "exit_field_access_expression"):
            listener.exit_field_access_expression(self)

    def accept(self, visitor):
        if hasattr(visitor, "visit_field_access_expression"):
            return visitor.visit_field_access_expression(self)
        else:
            return visitor.visit_children(self)


class BinaryExpression(ModelExpression):

    def __init__(self, operator):
        super().__init__()
        self.operator = operator

    def enter_node(self, listener):
        if hasattr(listener, "enter_binary_expression"):
            listener.enter_binary_expression(self)

    def exit_node(self, listener):
        if hasattr(listener, "exit_binary_expression"):
            listener.exit_binary_expression(self)

    def accept(self, visitor):
        if hasattr(visitor, "visit_binary_expression"):
            return visitor.visit_binary_expression(self)
        else:
            return visitor.visit_children(self)

    def get_type_string(self):
        type_string = None
        for child in self.get_children():
            current = child.get_type_string()
            if self.operator in ("/", "%", "*"):  # multiplied by factor
                if type_string is None or type_string in ("float", "int"):
                    type_string = current
            else:
                if type_string not in (current, type_string):
                    raise OSC2ParsingError(f'Children have different types {current}, {type_string}', context=self.get_ctx())
                type_string = current
        return type_string

    def get_resolved_value(self, blackboard=None):
        return visit_expression(self, blackboard).eval()


class UnaryExpression(ModelExpression):

    def __init__(self, operator):
        super().__init__()
        self.operator = operator

    def enter_node(self, listener):
        if hasattr(listener, "enter_unary_expression"):
            listener.enter_unary_expression(self)

    def exit_node(self, listener):
        if hasattr(listener, "exit_unary_expression"):
            listener.exit_unary_expression(self)

    def accept(self, visitor):
        if hasattr(visitor, "visit_unary_expression"):
            return visitor.visit_unary_expression(self)
        else:
            return visitor.visit_children(self)


class TernaryExpression(ModelExpression):

    def __init__(self):
        super().__init__()

    def enter_node(self, listener):
        if hasattr(listener, "enter_ternary_expression"):
            listener.enter_ternary_expression(self)

    def exit_node(self, listener):
        if hasattr(listener, "exit_ternary_expression"):
            listener.exit_ternary_expression(self)

    def accept(self, visitor):
        if hasattr(visitor, "visit_ternary_expression"):
            return visitor.visit_ternary_expression(self)
        else:
            return visitor.visit_children(self)


class LogicalExpression(ModelExpression):

    def __init__(self, operator):
        super().__init__()
        self.operator = operator

    def enter_node(self, listener):
        if hasattr(listener, "enter_logical_expression"):
            listener.enter_logical_expression(self)

    def exit_node(self, listener):
        if hasattr(listener, "exit_logical_expression"):
            listener.exit_logical_expression(self)

    def accept(self, visitor):
        if hasattr(visitor, "visit_logical_expression"):
            return visitor.visit_logical_expression(self)
        else:
            return visitor.visit_children(self)

    def get_type_string(self):
        return "bool"

    def get_resolved_value(self, blackboard=None):
        return visit_expression(self, blackboard).eval()


class RelationExpression(ModelExpression):

    def __init__(self, operator):
        super().__init__()
        self.operator = operator

    def enter_node(self, listener):
        if hasattr(listener, "enter_relation_expression"):
            listener.enter_relation_expression(self)

    def exit_node(self, listener):
        if hasattr(listener, "exit_relation_expression"):
            listener.exit_relation_expression(self)

    def accept(self, visitor):
        if hasattr(visitor, "visit_relation_expression"):
            return visitor.visit_relation_expression(self)
        else:
            return visitor.visit_children(self)

    def get_type_string(self):
        return "bool"

    def get_resolved_value(self, blackboard=None):
        return visit_expression(self, blackboard).eval()


class ListExpression(ModelExpression):

    def __init__(self):
        super().__init__()

    def enter_node(self, listener):
        if hasattr(listener, "enter_list_expression"):
            listener.enter_list_expression(self)

    def exit_node(self, listener):
        if hasattr(listener, "exit_list_expression"):
            listener.exit_list_literal(self)

    def accept(self, visitor):
        if hasattr(visitor, "visit_list_expression"):
            return visitor.visit_list_expression(self)
        else:
            return visitor.visit_children(self)

    def get_type_string(self):
        child = None
        if self.get_child_count():
            child = self.get_child(0)
        if not child:
            raise OSC2ParsingError(msg='At least on child expected.', context=self.get_ctx())
        type_string = child.get_type_string()
        return "listof" + type_string

    def get_resolved_value(self, blackboard=None):
        value = []
        for child in self.get_children():
            value.append(child.get_resolved_value(blackboard))
        return value


class RangeExpression(ModelExpression):

    def __init__(self):
        super().__init__()

    def enter_node(self, listener):
        if hasattr(listener, "enter_range_expression"):
            listener.enter_range_expression(self)

    def exit_node(self, listener):
        if hasattr(listener, "exit_range_expression"):
            listener.exit_range_expression(self)

    def accept(self, visitor):
        if hasattr(visitor, "visit_range_expression"):
            return visitor.visit_range_expression(self)
        else:
            return visitor.visit_children(self)


class PhysicalLiteral(ModelElement):

    def __init__(self, unit, value):
        super().__init__()
        self.value = value
        self.unit = unit

    def enter_node(self, listener):
        if hasattr(listener, "enter_physical_literal"):
            listener.enter_physical_literal(self)

    def exit_node(self, listener):
        if hasattr(listener, "exit_physical_literal"):
            listener.exit_physical_literal(self)

    def accept(self, visitor):
        if hasattr(visitor, "visit_physical_literal"):
            return visitor.visit_physical_literal(self)
        else:
            return visitor.visit_children(self)

    def get_value_child(self):
        return self.get_child(0)

    def get_type_string(self):
        return self.unit.get_type_string()

    def get_resolved_value(self, blackboard=None):
        si_unit_specifier = self.unit.find_first_child_of_type(SIUnitSpecifier)
        return self.get_value_child().get_resolved_value(blackboard) * si_unit_specifier.factor


class IntegerLiteral(ModelElement):

    def __init__(self, type_def, value):
        super().__init__()
        self.type_def = type_def  # uint, hex, int
        self.value = value

    def enter_node(self, listener):
        if hasattr(listener, "enter_integer_literal"):
            listener.enter_integer_literal(self)

    def exit_node(self, listener):
        if hasattr(listener, "exit_integer_literal"):
            listener.exit_integer_literal(self)

    def accept(self, visitor):
        if hasattr(visitor, "visit_integer_literal"):
            return visitor.visit_integer_literal(self)
        else:
            return visitor.visit_children(self)

    def get_type_string(self):
        return 'int'

    def get_resolved_value(self, blackboard=None):
        return int(self.value)


class FloatLiteral(ModelElement):

    def __init__(self, value):
        super().__init__()
        self.value = value

    def enter_node(self, listener):
        if hasattr(listener, "enter_float_literal"):
            listener.enter_float_literal(self)

    def exit_node(self, listener):
        if hasattr(listener, "exit_float_literal"):
            listener.exit_float_literal(self)

    def accept(self, visitor):
        if hasattr(visitor, "visit_float_literal"):
            return visitor.visit_float_literal(self)
        else:
            return visitor.visit_children(self)

    def get_type_string(self):
        return 'float'

    def get_resolved_value(self, blackboard=None):
        return float(self.value)


class BoolLiteral(ModelElement):

    def __init__(self, value):
        super().__init__()
        self.value = value

    def enter_node(self, listener):
        if hasattr(listener, "enter_bool_literal"):
            listener.enter_bool_literal(self)

    def exit_node(self, listener):
        if hasattr(listener, "exit_bool_literal"):
            listener.exit_bool_literal(self)

    def accept(self, visitor):
        if hasattr(visitor, "visit_bool_literal"):
            return visitor.visit_bool_literal(self)
        else:
            return visitor.visit_children(self)

    def get_type_string(self):
        return 'bool'

    def get_resolved_value(self, blackboard=None):
        return self.value == "true"


class StringLiteral(ModelElement):

    def __init__(self, value):
        super().__init__()
        self.value = value

    def get_value_child(self):
        return self.value

    def get_resolved_value(self, blackboard=None):
        return self.value.strip("\'").strip("\"")

    def enter_node(self, listener):
        if hasattr(listener, "enter_string_literal"):
            listener.enter_string_literal(self)

    def exit_node(self, listener):
        if hasattr(listener, "exit_string_literal"):
            listener.exit_string_literal(self)

    def accept(self, visitor):
        if hasattr(visitor, "visit_string_literal"):
            return visitor.visit_string_literal(self)
        else:
            return visitor.visit_children(self)

    def get_type_string(self):
        return 'string'


class Type(ModelElement):

    def __init__(self, type_def, is_list):
        super().__init__()
        self.type_def = type_def
        self.is_list = is_list

    def enter_node(self, listener):
        if hasattr(listener, "enter_type"):
            listener.enter_type(self)

    def exit_node(self, listener):
        if hasattr(listener, "exit_type"):
            listener.exit_type(self)

    def accept(self, visitor):
        if hasattr(visitor, "visit_type"):
            return visitor.visit_type(self)
        else:
            return visitor.visit_children(self)

    def get_resolved_value(self, blackboard=None):
        return None


class Identifier(ModelElement):

    def __init__(self, name):
        super().__init__()
        self.name = name

    def enter_node(self, listener):
        if hasattr(listener, "enter_identifier"):
            listener.enter_identifier(self)

    def exit_node(self, listener):
        if hasattr(listener, "exit_identifier"):
            listener.exit_identifier(self)

    def accept(self, visitor):
        if hasattr(visitor, "visit_identifier"):
            return visitor.visit_identifier(self)
        else:
            return visitor.visit_children(self)


class VariableReference(object):
    # To access variables from within action implementations

    def __init__(self, blackboard, ref) -> None:
        self.blackboard = blackboard
        self.ref = ref
        self.blackboard.register_key(ref, access=py_trees.common.Access.WRITE)

    def __str__(self):
        return f"VariableReference({self.ref})"

    def set_value(self, val):
        setattr(self.blackboard, self.ref, val)

    def get_value(self):
        return getattr(self.blackboard, self.ref)


class IdentifierReference(ModelElement):

    def __init__(self, ref):
        super().__init__()
        self.ref = ref

    def enter_node(self, listener):
        if hasattr(listener, "enter_identifier_reference"):
            listener.enter_identifier_reference(self)

    def exit_node(self, listener):
        if hasattr(listener, "exit_identifier_reference"):
            listener.exit_identifier_reference(self)

    def accept(self, visitor):
        if hasattr(visitor, "visit_identifier_reference"):
            return visitor.visit_identifier_reference(self)
        else:
            return visitor.visit_children(self)

    def get_type(self):
        if isinstance(self.ref, list):
            return self.ref[-1].get_type()
        else:
            return self.ref.get_type()

    def get_type_string(self):
        if isinstance(self.ref, list):
            return self.ref[-1].get_type_string()
        else:
            return self.ref.get_type_string()

    def get_blackboard_reference(self, blackboard):
        if not isinstance(self.ref, list) or len(self.ref) == 0:
            raise ValueError("Variable Reference only supported if reference is list with at least one element")
        if not isinstance(self.ref[0], ParameterDeclaration):
            raise ValueError("Variable Reference only supported if reference is part of a parameter declaration")
        fqn = self.ref[0].get_fully_qualified_var_name(include_scenario=False)
        if blackboard is None:
            raise ValueError("Variable Reference found, but no blackboard client available.")
        for sub_elem in self.ref[1:]:
            fqn += "/" + sub_elem.name
        blackboard.register_key(fqn, access=py_trees.common.Access.WRITE)
        return VariableReference(blackboard, fqn)

    def get_variable_reference(self, blackboard):
        if isinstance(self.ref, list) and any(isinstance(x, VariableDeclaration) for x in self.ref):
            return self.get_blackboard_reference(blackboard)
        else:
            return None

    def get_resolved_value(self, blackboard=None):
        if isinstance(self.ref, list):
            ref = self.ref[0]
            if any(isinstance(x, VariableDeclaration) for x in self.ref):
                var_ref = self.get_blackboard_reference(blackboard)
                return var_ref.get_value()
            else:
                val = ref.get_resolved_value(blackboard)
                for sub_elem in self.ref[1:]:
                    if sub_elem.name not in val:
                        if sub_elem != self.ref[-1]:
                            raise OSC2ParsingError(msg=f'Reference to {sub_elem.name} not found', context=self.get_ctx())
                    val = val[sub_elem.name]
                return val
        else:
            return self.ref.get_resolved_value(blackboard)


class Expression(object):
    def __init__(self, left, right, operator) -> None:
        self.left = left
        self.right = right
        self.operator = operator

    def resolve(self, param, blackboard):
        if isinstance(param, Expression):
            return param.eval(blackboard)
        elif isinstance(param, VariableReference):
            return param.get_value()
        elif isinstance(param, FunctionApplicationExpression):
            return param.get_resolved_value(blackboard)
        else:
            return param

    def eval(self, blackboard):
        left = self.resolve(self.left, blackboard)
        if self.right is None:
            return self.operator(left)
        else:
            right = self.resolve(self.right, blackboard)
            return self.operator(left, right)


def visit_expression(node, blackboard):
    operator = None
    single_child = False
    if node.operator == "==":
        operator = op.eq
    elif node.operator == "!=":
        operator = op.ne
    elif node.operator == "<":
        operator = op.lt
    elif node.operator == "<=":
        operator = op.le
    elif node.operator == ">":
        operator = op.gt
    elif node.operator == ">=":
        operator = op.ge
    elif node.operator == "and":
        operator = op.and_
    elif node.operator == "or":
        operator = op.or_
    elif node.operator == "not":
        single_child = True
        operator = op.not_
    elif node.operator == "+":
        operator = op.add
    elif node.operator == "-":
        operator = op.sub
    elif node.operator == "*":
        operator = op.mul
    elif node.operator == "/":
        operator = op.truediv
    elif node.operator == "%":
        operator = op.mod
    else:
        raise NotImplementedError(f"Unknown expression operator '{node.operator}'.")

    if not single_child and node.get_child_count() != 2:
        raise ValueError("Expression is expected to have two children.")

    idx = 0
    args = [None, None]
    for child in node.get_children():
        if isinstance(child, (RelationExpression, BinaryExpression, LogicalExpression)):
            args[idx] = visit_expression(child, blackboard)
        else:
            if isinstance(child, IdentifierReference):
                var_def = child.get_variable_reference(blackboard)
                if var_def is not None:
                    args[idx] = var_def
                else:
                    args[idx] = child.get_resolved_value(blackboard)
            elif isinstance(child, FunctionApplicationExpression):
                args[idx] = child
            else:
                args[idx] = child.get_resolved_value(blackboard)
        idx += 1

    if single_child:
        return Expression(args[0], args[1], operator)
    else:
        return Expression(args[0], args[1], operator)
