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

"""
Test parsing error reporting for features that are not yet supported
"""
import unittest

from scenario_execution_base.model.osc2_parser import OpenScenario2Parser
from scenario_execution_base.utils.logging import Logger
from antlr4.InputStream import InputStream


class TestOSC2Parser(unittest.TestCase):
    # pylint: disable=missing-function-docstring, protected-access, no-member, unused-variable, too-many-public-methods

    def setUp(self) -> None:
        self.parser = OpenScenario2Parser(Logger('test', False))

    def test_method(self):
        scenario_content = """
actor test:
    def get_value() -> float is external TEST()
"""
        parsed_tree = self.parser.parse_input_stream(InputStream(scenario_content))
        self.assertRaises(ValueError, self.parser.create_internal_model, parsed_tree, "test.osc")

    def test_cover(self):
        scenario_content = """
scenario test:
    cover()
"""
        parsed_tree = self.parser.parse_input_stream(InputStream(scenario_content))
        self.assertRaises(ValueError, self.parser.create_internal_model, parsed_tree, "test.osc")

    def test_lt_expr(self):
        scenario_content = """
actor car:
    current_position: float
    keep(current_position < 100.0)
"""
        parsed_tree = self.parser.parse_input_stream(InputStream(scenario_content))
        self.assertRaises(ValueError, self.parser.create_internal_model, parsed_tree, "test.osc")

    def test_gt_expr(self):
        scenario_content = """
actor car:
    current_position: float
    keep(current_position > 100.0)
"""
        parsed_tree = self.parser.parse_input_stream(InputStream(scenario_content))
        self.assertRaises(ValueError, self.parser.create_internal_model, parsed_tree, "test.osc")

    def test_le_expr(self):
        scenario_content = """
actor car:
    current_position: float
    keep(current_position <= 100.0)
"""
        parsed_tree = self.parser.parse_input_stream(InputStream(scenario_content))
        self.assertRaises(ValueError, self.parser.create_internal_model, parsed_tree, "test.osc")

    def test_ge_expr(self):
        scenario_content = """
actor car:
    current_position: float
    keep(current_position >= 100.0)
"""
        parsed_tree = self.parser.parse_input_stream(InputStream(scenario_content))
        self.assertRaises(ValueError, self.parser.create_internal_model, parsed_tree, "test.osc")

    def test_range(self):
        scenario_content = """
global test: float = range(0,1)
"""
        parsed_tree = self.parser.parse_input_stream(InputStream(scenario_content))
        self.assertRaises(ValueError, self.parser.create_internal_model, parsed_tree, "test.osc")

    def test_element_access(self):
        scenario_content = """
global foo: list of float
global test: float = foo[2]
"""
        parsed_tree = self.parser.parse_input_stream(InputStream(scenario_content))
        self.assertRaises(ValueError, self.parser.create_internal_model, parsed_tree, "test.osc")

    def test_type_test(self):
        scenario_content = """
global foo: float
global test: bool = foo.is(float)
"""
        parsed_tree = self.parser.parse_input_stream(InputStream(scenario_content))
        self.assertRaises(ValueError, self.parser.create_internal_model, parsed_tree, "test.osc")

    def test_cast(self):
        scenario_content = """
global foo: float
global test: int = foo.as(int)
"""
        parsed_tree = self.parser.parse_input_stream(InputStream(scenario_content))
        self.assertRaises(ValueError, self.parser.create_internal_model, parsed_tree, "test.osc")

    def test_sample(self):
        scenario_content = """
import osc.standard.base

scenario simple_drive:    
    environment: environment
    var sim_start_time: time = sample(environment.datetime, @root_phase.start)
"""
        parsed_tree = self.parser.parse_input_stream(InputStream(scenario_content))
        self.assertRaises(ValueError, self.parser.create_internal_model, parsed_tree, "test.osc")

    def test_every(self):
        scenario_content = """
scenario test:
    event test is every(1)
"""
        parsed_tree = self.parser.parse_input_stream(InputStream(scenario_content))
        self.assertRaises(ValueError, self.parser.create_internal_model, parsed_tree, "test.osc")

    def test_rise(self):
        scenario_content = """
scenario test:
    event test is rise(true)
"""
        parsed_tree = self.parser.parse_input_stream(InputStream(scenario_content))
        self.assertRaises(ValueError, self.parser.create_internal_model, parsed_tree, "test.osc")

    def test_fall(self):
        scenario_content = """
scenario test:
    event test is fall(true)
"""
        parsed_tree = self.parser.parse_input_stream(InputStream(scenario_content))
        self.assertRaises(ValueError, self.parser.create_internal_model, parsed_tree, "test.osc")

    def test_modifier(self):
        scenario_content = """
modifier test
"""
        parsed_tree = self.parser.parse_input_stream(InputStream(scenario_content))
        self.assertRaises(ValueError, self.parser.create_internal_model, parsed_tree, "test.osc")

    def test_modifier_application(self):
        scenario_content = """
modifier test_modifier:
    param1: string

action test_action:
    param2: string

scenario test:
    do serial:
        test_action() with:
            test_modifier(6)
"""
        parsed_tree = self.parser.parse_input_stream(InputStream(scenario_content))
        self.assertRaises(ValueError, self.parser.create_internal_model, parsed_tree, "test.osc")

    def test_until(self):
        scenario_content = """
scenario test:
    event ev
    do serial:
        test_action() with:
            until @ev
"""
        parsed_tree = self.parser.parse_input_stream(InputStream(scenario_content))
        self.assertRaises(ValueError, self.parser.create_internal_model, parsed_tree, "test.osc")

    def test_call(self):
        scenario_content = """
scenario test:
    do serial:
        call log(true)
"""
        parsed_tree = self.parser.parse_input_stream(InputStream(scenario_content))
        self.assertRaises(ValueError, self.parser.create_internal_model, parsed_tree, "test.osc")

    def test_remove_default(self):
        scenario_content = """
struct base:
    param_base: string = "base"
    
struct test:
    param1: base = base with:
        remove_default(it.param_base)
"""
        parsed_tree = self.parser.parse_input_stream(InputStream(scenario_content))
        self.assertRaises(ValueError, self.parser.create_internal_model, parsed_tree, "test.osc")

    def test_record(self):
        scenario_content = """
scenario test:
    record()
"""
        parsed_tree = self.parser.parse_input_stream(InputStream(scenario_content))
        self.assertRaises(ValueError, self.parser.create_internal_model, parsed_tree, "test.osc")

    def test_enum_extend(self):
        scenario_content = """
enum test: [a, b]
extend test: [c]
"""
        parsed_tree = self.parser.parse_input_stream(InputStream(scenario_content))
        self.assertRaises(ValueError, self.parser.create_internal_model, parsed_tree, "test.osc")

    def test_conditional_inheritance(self):
        scenario_content = """
struct vehicle:
    is_electric: bool

actor electric_vehicle inherits vehicle(is_electric == true)
"""
        parsed_tree = self.parser.parse_input_stream(InputStream(scenario_content))
        self.assertRaises(ValueError, self.parser.create_internal_model, parsed_tree, "test.osc")
