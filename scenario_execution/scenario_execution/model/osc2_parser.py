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
from scenario_execution.model.types import print_tree
from scenario_execution.model.model_to_py_tree import create_py_tree
from scenario_execution.model.model_resolver import resolve_internal_model


class OpenScenario2Parser(object):
    """ Helper class for parsing openscenario 2 files """

    def __init__(self, logger) -> None:
        self.logger = logger
        self.parsed_files = []

    def process_file(self, file, log_model: bool = False, debug: bool = False):
        """ Convenience method to execute the parsing and print out tree """

        parsed_tree = self.parse_file(file, log_model)

        model = self.create_internal_model(parsed_tree, file, log_model, debug)

        scenarios = create_py_tree(model, self.logger, log_model)

        return scenarios

    def load_internal_model(self, tree, file_name: str, log_model: bool = False, debug: bool = False):
        model_builder = ModelBuilder(self.logger, self.parse_file, file_name, log_model)
        walker = ParseTreeWalker()

        model = None
        try:
            walker.walk(model_builder, tree)
            model = model_builder.get_model()
        except OSC2ParsingError as e:
            raise ValueError(
                f'Error creating internal model: Traceback <line: {e.line}, column: {e.column}> in "{e.filename}":\n  -> {e.context}\n{e.__class__.__name__}: {e.msg}') from e
        if log_model:
            self.logger.info("----Internal model-----")
            print_tree(model, self.logger)
        return model

    def create_internal_model(self, tree, file_name: str, log_model: bool = False, debug: bool = False):
        model = self.load_internal_model(tree, file_name, log_model, debug)
        resolve_internal_model(model, self.logger, log_model)
        return model

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
