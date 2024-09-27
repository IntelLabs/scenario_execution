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

import re

from antlr4 import FileStream, CommonTokenStream
from antlr4.error.ErrorListener import ErrorListener
from antlr4.tree.Tree import TerminalNodeImpl, ParseTreeWalker
from scenario_execution.osc2_parsing.OpenSCENARIO2Parser import OpenSCENARIO2Parser
from scenario_execution.osc2_parsing.OpenSCENARIO2Lexer import OpenSCENARIO2Lexer
from scenario_execution.model.error import OSC2ParsingError
from scenario_execution.model.model_builder import ModelBuilder
from scenario_execution.model.types import print_tree, ScenarioDeclaration, ParameterDeclaration, StringLiteral, FloatLiteral, BoolLiteral, IntegerLiteral, PhysicalTypeDeclaration, PhysicalLiteral, StructDeclaration, FunctionApplicationExpression, IdentifierReference, NamedArgument, PositionalArgument
from scenario_execution.model.model_to_py_tree import create_py_tree
from scenario_execution.model.model_resolver import resolve_internal_model
from scenario_execution.model.model_blackboard import create_py_tree_blackboard
import py_trees


class OpenScenario2Parser(object):
    """ Helper class for parsing openscenario 2 files """

    def __init__(self, logger) -> None:
        self.logger = logger
        self.parsed_files = []

    def process_file(self, file, log_model: bool = False, debug: bool = False, scenario_parameter_overrides: dict = None):
        """ Convenience method to execute the parsing and print out tree """

        parsed_model = self.parse_file(file, log_model)

        tree = py_trees.composites.Sequence(name="", memory=True)
        model = self.create_internal_model(parsed_model, tree, file, log_model, debug, scenario_parameter_overrides)

        if len(model.find_children_of_type(ScenarioDeclaration)) == 0:
            raise ValueError("No scenario defined.")

        if len(model.find_children_of_type(ScenarioDeclaration)) != 1:
            raise ValueError("More than one scenario defined.")

        create_py_tree_blackboard(model, tree, self.logger, debug)

        return create_py_tree(model, tree, self.logger, log_model)

    def load_internal_model(self, tree, file_name: str, log_model: bool = False, debug: bool = False):
        model_builder = ModelBuilder(self.logger, self.parse_file, file_name, log_model)
        walker = ParseTreeWalker()

        model = None
        try:
            walker.walk(model_builder, tree)
            model = model_builder.get_model()
        except OSC2ParsingError as e:
            raise ValueError(f'Error creating internal model: {e}') from e
        if log_model:
            self.logger.info("----Internal model-----")
            print_tree(model, self.logger)
        return model

    def create_internal_model(self, parsed_model, tree, file_name: str, log_model: bool = False, debug: bool = False, scenario_parameter_overrides: dict = None):
        model = self.load_internal_model(parsed_model, file_name, log_model, debug)
        resolve_internal_model(model, tree, self.logger, log_model)
        
        # override parameter with externally defined ones
        if scenario_parameter_overrides:
            keys = list(scenario_parameter_overrides.keys())
            for scenario in model.find_children_of_type(ScenarioDeclaration):
                if scenario.name in keys:
                    keys.remove(scenario.name)
                    if scenario_parameter_overrides[scenario.name] is None:
                        continue
                    param_keys = list(scenario_parameter_overrides[scenario.name].keys())
                    for parameter in scenario.find_children_of_type(ParameterDeclaration):
                        if parameter.name in param_keys:
                            param_keys.remove(parameter.name)
                            override_value = scenario_parameter_overrides[scenario.name][parameter.name]
                            child_def = parameter.get_value_child()
                            try:
                                self.set_override_value(child_def, override_value)
                            except ValueError as e:
                                raise ValueError(f"{parameter.name} {e}") from e
                                
            if len(keys) > 0:
                raise ValueError(f"Scenario Parameter Overrides contain unknown scenario(s): {', '.join(keys)}")
        return model


    def set_override_value_function_application(self, parameter, override_value):
        idx = 0
        struct_keys = list(override_value.keys())
        pos = 0
        for child in parameter.get_children():
            if idx == 0:
                if not isinstance(child, IdentifierReference):
                    raise ValueError(f"Expected IdentifierReference, got {child}")
                ref = child.ref
            else:
                arg_name = None
                if isinstance(child, NamedArgument):
                    arg_name = child.name
                elif isinstance(child, PositionalArgument):
                    arg_name = ref.get_child(pos).name
                    pos += 1
                
                if arg_name:
                    struct_keys.remove(arg_name)
                    child_app = child.get_only_child()
                    try:
                        self.set_override_value(child_app, override_value[arg_name])
                    except ValueError as e:
                        raise ValueError(f"{arg_name}: {e}") from e                    
            idx += 1
        if struct_keys:
            raise ValueError(f"Unknown override values found: {', '.join(struct_keys)}")
    
    def set_override_value(self, param, override_value):
        if isinstance(param, FunctionApplicationExpression):
            self.set_override_value_function_application(param, override_value)
        elif isinstance(param, StringLiteral):
            param.value = str(override_value)
        elif isinstance(param, BoolLiteral):
            if not isinstance(override_value, (bool)):
                raise ValueError(f"bool expected, found {type(override_value).__name__}")
            param.value = override_value
        elif isinstance(param, FloatLiteral):
            if not isinstance(override_value, (int, float)):
                raise ValueError(f"float or int expected, found {type(override_value).__name__}")
            param.value = override_value
        elif isinstance(param, IntegerLiteral):
            if not isinstance(override_value, int):
                raise ValueError(f"integer expected, found {type(override_value).__name__}")
            param.value = override_value
        elif isinstance(param, PhysicalLiteral):
            literal = param.find_first_child_of_type((FloatLiteral, IntegerLiteral))
            if isinstance(literal, FloatLiteral) and isinstance(override_value, (int, float)):
                literal.value = float(override_value)
            elif isinstance(literal, IntegerLiteral) and isinstance(override_value, int):
                literal.value = override_value
            else:
                raise ValueError(f"Invalid physical literal.")
        else:
            raise ValueError(f"(AS)")
        
    def parse_file(self, file: str, log_model: bool = False, error_prefix=""):
        """ Execute the parsing """
        if file in self.parsed_files:  # skip already parsed/imported files
            return None
        self.parsed_files.append(file)
        try:
            input_stream = FileStream(file)
        except (OSError, UnicodeDecodeError) as e:
            raise ValueError(f'{e}') from e
        return self.parse_input_stream(input_stream, log_model, error_prefix)

    def parse_input_stream(self, input_stream, log_model=False, error_prefix=""):
        """ Execute the parsing """
        lexer = OpenSCENARIO2Lexer(input_stream)
        stream = CommonTokenStream(lexer)

        parser = OpenSCENARIO2Parser(stream)
        # if quiet:
        parser.removeErrorListeners()

        class TestErrorListener(ErrorListener):
            def __init__(self, prefix: str) -> None:
                self.prefix = prefix
                self.error_message = ""
                super().__init__()

            def syntaxError(self, recognizer, offendingSymbol, line, column, msg, e):  # pylint: disable=invalid-name
                if self.error_message:
                    self.error_message += "\n"
                self.error_message += self.prefix + "line " + str(line) + ":" + str(column) + " " + msg
        error_listener = TestErrorListener(error_prefix)
        parser.addErrorListener(error_listener)
        tree = parser.osc_file()
        errors = parser.getNumberOfSyntaxErrors()  # pylint: disable=no-member
        if log_model:
            self.print_parsed_osc_tree(tree, self.logger, parser.ruleNames)
        if errors:
            raise ValueError(error_listener.error_message)
        del parser
        return tree

    @staticmethod
    def print_parsed_osc_tree(tree, logger, rule_names, indent=0):
        """ Print the parsed tree for debugging purposes """
        if isinstance(tree, TerminalNodeImpl):
            if not re.match(r"\r?\n[ \t]*", tree.getText()):
                logger.info("{0}TOKEN '{1}'".format("  " * indent, tree.getText()))
        else:
            logger.info("{0}{1}".format("  " * indent, rule_names[tree.getRuleIndex()]))
            if tree.children:
                for child in tree.children:
                    OpenScenario2Parser.print_parsed_osc_tree(child, logger, rule_names, indent+1)
