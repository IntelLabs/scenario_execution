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
Test struct parsing
"""
import unittest

from scenario_execution_base.model.osc2_parser import OpenScenario2Parser
from scenario_execution_base.utils.logging import Logger
from antlr4.InputStream import InputStream


class TestOSC2Parser(unittest.TestCase):
    # pylint: disable=missing-function-docstring, protected-access, no-member, unused-variable

    def setUp(self) -> None:
        self.parser = OpenScenario2Parser(Logger('test', False))

    def test_all_struct_def(self):
        scenario_content = """
global level1: string = "hey!"
global level2: string = level1
global override: string = "override"

struct base_struct:
    base_param1: string = level2

struct struct1a:
    struct_param: base_struct

struct struct1b:
    struct_param: base_struct = base_struct()

struct struct1c:
    struct_param: base_struct = base_struct
    
struct struct2a:
    struct_param: base_struct = base_struct(base_param1: "override")

struct struct2b:
    struct_param: base_struct = base_struct(base_param1: override)

struct struct2c:
    struct_param: base_struct = base_struct("override")
    
struct struct2d:
    struct_param: base_struct = base_struct(override)
    
scenario test:
    struct2e: struct1a with:
        keep(it.struct_param == base_struct('override'))

    struct2f: struct1a with:
        keep(it.struct_param == base_struct(base_param1: 'override'))
"""
        parsed_tree, errors = self.parser.parse_input_stream(InputStream(scenario_content))
        self.assertEqual(errors, 0)
        model = self.parser.create_internal_model(parsed_tree, "test.osc", True)
        self.assertIsNotNone(model)

        struct1a = model._ModelElement__children[4].get_resolved_value()
        struct1b = model._ModelElement__children[5].get_resolved_value()
        struct1c = model._ModelElement__children[6].get_resolved_value()
        struct2a = model._ModelElement__children[7].get_resolved_value()
        struct2b = model._ModelElement__children[8].get_resolved_value()
        struct2c = model._ModelElement__children[9].get_resolved_value()
        struct2d = model._ModelElement__children[10].get_resolved_value()
        struct2e = model._ModelElement__children[11]._ModelElement__children[0].get_resolved_value()
        struct2f = model._ModelElement__children[11]._ModelElement__children[1].get_resolved_value()
        self.assertEqual({'struct_param': {'base_param1': 'hey!'}}, struct1a)
        self.assertEqual({'struct_param': {'base_param1': 'hey!'}}, struct1b)
        self.assertEqual({'struct_param': {'base_param1': 'hey!'}}, struct1c)
        self.assertEqual({'struct_param': {'base_param1': 'override'}}, struct2a)
        self.assertEqual({'struct_param': {'base_param1': 'override'}}, struct2b)
        self.assertEqual({'struct_param': {'base_param1': 'override'}}, struct2c)
        self.assertEqual({'struct_param': {'base_param1': 'override'}}, struct2d)
        self.assertEqual({'struct_param': {'base_param1': 'override'}}, struct2e)
        self.assertEqual({'struct_param': {'base_param1': 'override'}}, struct2f)

    def test_multi_layer(self):
        scenario_content = """
struct base_struct:
    param1: string = 'test'

struct l1_struct:
    base: base_struct

struct l2_struct:
    l1: l1_struct
    
scenario test:
    struct1: l2_struct with:
        keep(it.l1 == l1_struct(base: base_struct))
        
    struct2: l2_struct with:
        keep(it.l1 == l1_struct(base: base_struct('override')))
        
    struct3: l2_struct with:
        keep(it.l1 == l1_struct(base_struct('override')))
        
    struct4: l2_struct with:
        keep(it.l1.base.param1 == 'override')
        
    struct5: l2_struct with:
        keep(it.l1.base == base_struct('override'))
"""
        parsed_tree, errors = self.parser.parse_input_stream(InputStream(scenario_content))
        self.assertEqual(errors, 0)
        model = self.parser.create_internal_model(parsed_tree, "test.osc", True)
        self.assertIsNotNone(model)

        struct1 = model._ModelElement__children[3]._ModelElement__children[0].get_resolved_value()
        struct2 = model._ModelElement__children[3]._ModelElement__children[1].get_resolved_value()
        struct3 = model._ModelElement__children[3]._ModelElement__children[2].get_resolved_value()
        struct4 = model._ModelElement__children[3]._ModelElement__children[3].get_resolved_value()
        struct5 = model._ModelElement__children[3]._ModelElement__children[4].get_resolved_value()
        self.assertEqual({'l1': {'base': {'param1': 'test'}}}, struct1)
        self.assertEqual({'l1': {'base': {'param1': 'override'}}}, struct2)
        self.assertEqual({'l1': {'base': {'param1': 'override'}}}, struct3)
        self.assertEqual({'l1': {'base': {'param1': 'override'}}}, struct4)
        self.assertEqual({'l1': {'base': {'param1': 'override'}}}, struct5)

    def test_sub_struct_assignment(self):
        scenario_content = """
struct base_struct:
    param1: string = 'test'

struct l1_struct:
    base: base_struct

struct l2_struct:
    l1: l1_struct
    
scenario test:        
    struct4: l2_struct with:
        keep(it.l1.base.param1 == 'override')
"""
        parsed_tree, errors = self.parser.parse_input_stream(InputStream(scenario_content))
        self.assertEqual(errors, 0)
        model = self.parser.create_internal_model(parsed_tree, "test.osc", True)
        self.assertIsNotNone(model)

        struct1 = model._ModelElement__children[3]._ModelElement__children[0].get_resolved_value()
        self.assertEqual({'l1': {'base': {'param1': 'override'}}}, struct1)

    def test_sub_struct_invalid_membername(self):
        scenario_content = """
struct base_struct:
    param1: string = 'test'

struct l1_struct:
    base: base_struct

struct l2_struct:
    l1: l1_struct
    
scenario test:        
    struct4: l2_struct with:
        keep(it.l1.UNKNOWN.param1 == 'override')
"""
        parsed_tree, errors = self.parser.parse_input_stream(InputStream(scenario_content))
        self.assertEqual(errors, 0)
        model = self.parser.create_internal_model(parsed_tree, "test.osc", True)
        self.assertIsNone(model)
