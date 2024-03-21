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
Test parameter parsing
"""
import unittest

from scenario_execution_base.model.osc2_parser import OpenScenario2Parser
from scenario_execution_base.model.model_to_py_tree import create_py_tree
from scenario_execution_base.utils.logging import Logger
from antlr4.InputStream import InputStream


class TestOSC2Parser(unittest.TestCase):
    # pylint: disable=missing-function-docstring, protected-access, no-member, unused-variable, too-many-public-methods
    """
    Unit test for osc2_parser
    """

    def setUp(self) -> None:
        self.parser = OpenScenario2Parser(Logger('test', False))

    def test_invalid(self):
        scenario_content = """
invalid
"""
        parsed_tree, errors = self.parser.parse_input_stream(InputStream(scenario_content))
        self.assertEqual(errors, 1)

    def test_empty(self):
        scenario_content = ""
        parsed_tree, errors = self.parser.parse_input_stream(InputStream(scenario_content))
        self.assertEqual(errors, 0)
        model = self.parser.create_internal_model(parsed_tree, "test.osc", True)
        self.assertIsNotNone(model)
        scenarios = create_py_tree(model, self.parser.logger, False)
        self.assertEqual([], scenarios)

    def test_global_var(self):
        scenario_content = """
global level1: string = "hey!"
"""
        parsed_tree, errors = self.parser.parse_input_stream(InputStream(scenario_content))
        self.assertEqual(errors, 0)

        model = self.parser.create_internal_model(parsed_tree, "test.osc", True)
        self.assertIsNotNone(model)
        param = model._ModelElement__children[0].get_resolved_value()
        self.assertEqual(param, "hey!")

    def test_global_var_indirect_1(self):
        scenario_content = """
global level1: string = "hey!"
global level2: string = level1
"""
        parsed_tree, errors = self.parser.parse_input_stream(InputStream(scenario_content))
        self.assertEqual(errors, 0)
        model = self.parser.create_internal_model(parsed_tree, "test.osc", True)
        self.assertIsNotNone(model)
        self.assertEqual(len(model._ModelElement__children), 2)
        glob_param1 = model._ModelElement__children[0]
        val1 = glob_param1.get_resolved_value()
        self.assertEqual(val1, "hey!")
        glob_param2 = model._ModelElement__children[1]
        val2 = glob_param2.get_resolved_value()
        self.assertEqual(val2, "hey!")

    def test_global_var_indirect_2(self):
        scenario_content = """
global level1: string = "hey!"
global level2: string = level1
global level3: string = level2
"""
        parsed_tree, errors = self.parser.parse_input_stream(InputStream(scenario_content))
        self.assertEqual(errors, 0)
        model = self.parser.create_internal_model(parsed_tree, "test.osc", True)
        self.assertIsNotNone(model)
        self.assertEqual(model._ModelElement__children[0].get_resolved_value(), 'hey!')
        self.assertEqual(model._ModelElement__children[1].get_resolved_value(), 'hey!')
        self.assertEqual(model._ModelElement__children[2].get_resolved_value(), 'hey!')

    def test_global_var_indirect_2_wrong_type(self):
        scenario_content = """
global level1: string = "hey!"
global level2: int = level1
global level3: string = level2
"""
        parsed_tree, errors = self.parser.parse_input_stream(InputStream(scenario_content))
        self.assertEqual(errors, 0)
        model = self.parser.create_internal_model(parsed_tree, "test.osc", True)
        self.assertIsNone(model)

    def test_global_var_indirect_3(self):
        scenario_content = """
global level1: string = "hey!"
global level2: string = level1
global level3: string = level2

scenario test:
    var test_var: string = level3
"""
        parsed_tree, errors = self.parser.parse_input_stream(InputStream(scenario_content))
        self.assertEqual(errors, 0)
        model = self.parser.create_internal_model(parsed_tree, "test.osc", True)
        self.assertIsNotNone(model)
        self.assertEqual(model._ModelElement__children[0].get_resolved_value(), 'hey!')
        self.assertEqual(model._ModelElement__children[1].get_resolved_value(), 'hey!')
        self.assertEqual(model._ModelElement__children[2].get_resolved_value(), 'hey!')
        self.assertEqual(model._ModelElement__children[3]._ModelElement__children[0].get_resolved_value(), 'hey!')

    def test_global_var_indirect_not_found(self):
        scenario_content = """
global level1: string = "hey!"
global level2: int = levelUNKNOWN
"""
        parsed_tree, errors = self.parser.parse_input_stream(InputStream(scenario_content))
        self.assertEqual(errors, 0)
        model = self.parser.create_internal_model(parsed_tree, "test.osc")
        self.assertIsNone(model)

    def test_global_var_indirect_in_struct(self):
        scenario_content = """
global z_param: string = "z_global"
struct test_struct:
    z: string = z_param
"""
        parsed_tree, errors = self.parser.parse_input_stream(InputStream(scenario_content))
        self.assertEqual(errors, 0)
        model = self.parser.create_internal_model(parsed_tree, "test.osc", True)
        self.assertIsNotNone(model)
        self.assertEqual({'z': 'z_global'}, model._ModelElement__children[1].get_resolved_value())

    def test_global_var_indirect_in_actor(self):
        scenario_content = """
global z_param: string = "z_global"
actor test_struct:
    z: string = z_param
"""
        parsed_tree, errors = self.parser.parse_input_stream(InputStream(scenario_content))
        self.assertEqual(errors, 0)
        model = self.parser.create_internal_model(parsed_tree, "test.osc", True)
        self.assertIsNotNone(model)
        self.assertEqual({'z': 'z_global'}, model._ModelElement__children[1].get_resolved_value())

    def test_var_instance_unknown_type(self):
        scenario_content = """
scenario test:
    test_pose: unknownType 
"""
        parsed_tree, errors = self.parser.parse_input_stream(InputStream(scenario_content))
        self.assertEqual(errors, 0)
        model = self.parser.create_internal_model(parsed_tree, "test.osc")
        self.assertIsNone(model)

    def test_var_instance_keep_not_it(self):
        scenario_content = """
struct pose3d:
    z: string = "base"
scenario network_drop_straight:
    test_pose: pose3d with:
        keep(NO.z == "override")
"""
        parsed_tree, errors = self.parser.parse_input_stream(InputStream(scenario_content))
        self.assertEqual(errors, 0)
        model = self.parser.create_internal_model(parsed_tree, "test.osc", True)
        self.assertIsNone(model)

    def test_var_instance_keep_unknown_member(self):
        scenario_content = """
struct pose3d:
    z: string = "base"
scenario network_drop_straight:
    test_pose: pose3d with:
        keep(it.UNKNOWN == "override")
"""
        parsed_tree, errors = self.parser.parse_input_stream(InputStream(scenario_content))
        self.assertEqual(errors, 0)
        model = self.parser.create_internal_model(parsed_tree, "test.osc", True)
        self.assertIsNone(model)

    def test_var_instance_keep_success(self):
        scenario_content = """
struct pose3d:
    z: string = "base"
scenario network_drop_straight:
    test_pose: pose3d with:
        keep(it.z == "override")
"""
        parsed_tree, errors = self.parser.parse_input_stream(InputStream(scenario_content))
        self.assertEqual(errors, 0)
        model = self.parser.create_internal_model(parsed_tree, "test.osc", True)
        self.assertIsNotNone(model)

    def test_var_instance_keep_not_equal(self):
        scenario_content = """
struct pose3d:
    z: string = "base"
scenario network_drop_straight:
    test_pose: pose3d with:
        keep(it.z <= "override")
"""
        parsed_tree, errors = self.parser.parse_input_stream(InputStream(scenario_content))
        self.assertEqual(errors, 0)
        model = self.parser.create_internal_model(parsed_tree, "test.osc", True)
        self.assertIsNone(model)

    def test_struct_inheritance(self):
        scenario_content = """
struct base:
    x: string = "base"
    
struct derived inherits base:
    y: string = "derived"
"""
        parsed_tree, errors = self.parser.parse_input_stream(InputStream(scenario_content))
        self.assertEqual(errors, 0)
        model = self.parser.create_internal_model(parsed_tree, "test.osc", True)
        self.assertIsNotNone(model)
        self.assertEqual({'x': 'base', 'y': 'derived'},
                         model._ModelElement__children[1].get_resolved_value())

    def test_actor_inheritance(self):
        scenario_content = """
actor base:
    x: string = "base"
    
actor derived inherits base:
    y: string = "derived"
"""
        parsed_tree, errors = self.parser.parse_input_stream(InputStream(scenario_content))
        self.assertEqual(errors, 0)
        model = self.parser.create_internal_model(parsed_tree, "test.osc", True)
        self.assertIsNotNone(model)
        self.assertEqual({'x': 'base', 'y': 'derived'},
                         model._ModelElement__children[1].get_resolved_value())

    def test_actor_inheritance_invalid(self):
        scenario_content = """
actor derived inherits UNKNOWN:
    y: string = "derived"
"""
        parsed_tree, errors = self.parser.parse_input_stream(InputStream(scenario_content))
        self.assertEqual(errors, 0)
        model = self.parser.create_internal_model(parsed_tree, "test.osc")
        self.assertIsNone(model)

        scenario_content = """
action base:
    x: string = "base"
    
action derived inherits base:
    y: string = "derived"
"""
        parsed_tree, errors = self.parser.parse_input_stream(InputStream(scenario_content))
        self.assertEqual(errors, 0)
        model = self.parser.create_internal_model(parsed_tree, "test.osc", True)
        self.assertIsNotNone(model)
        self.assertEqual({'x': 'base', 'y': 'derived'},
                         model._ModelElement__children[1].get_resolved_value())

    def test_action_inheritance_invalid(self):
        scenario_content = """
action derived inherits UNKNOWN:
    y: string = "derived"
"""
        parsed_tree, errors = self.parser.parse_input_stream(InputStream(scenario_content))
        self.assertEqual(errors, 0)
        model = self.parser.create_internal_model(parsed_tree, "test.osc")
        self.assertIsNone(model)

    def test_scenario_inheritance(self):
        scenario_content = """
scenario base:
    x: string = "base"
    
scenario derived inherits base:
    y: string = "derived"
"""
        parsed_tree, errors = self.parser.parse_input_stream(InputStream(scenario_content))
        self.assertEqual(errors, 0)
        model = self.parser.create_internal_model(parsed_tree, "test.osc", True)
        self.assertIsNotNone(model)
        derived = model._ModelElement__children[1]
        self.assertEqual({'x': 'base', 'y': 'derived'}, derived.get_resolved_value())

    def test_scenario_inheritance_invalid(self):
        scenario_content = """
scenario derived inherits UNKNOWN:
    y: string = "derived"
"""
        parsed_tree, errors = self.parser.parse_input_stream(InputStream(scenario_content))
        self.assertEqual(errors, 0)
        model = self.parser.create_internal_model(parsed_tree, "test.osc", True)
        self.assertIsNone(model)

    def test_parameter_to_dict(self):
        scenario_content = """
struct test_struct:
    x: string = "value_x"
    y: string = "value_y"
    
scenario test:
    foo: test_struct with:
        keep(it.x == "override_x")
"""
        parsed_tree, errors = self.parser.parse_input_stream(InputStream(scenario_content))
        self.assertEqual(errors, 0)
        model = self.parser.create_internal_model(parsed_tree, "test.osc", True)
        self.assertIsNotNone(model)
        param = model._ModelElement__children[1]._ModelElement__children[0]
        values = param.get_resolved_value()
        self.assertEqual({"x": "override_x", "y": "value_y"}, values)

    def test_struct_with_keep(self):
        scenario_content = """
struct test_struct:
    x: string = "value_x"
    y: string = "value_y"
    
global foo: test_struct with:
        keep(it.x == "override_x")
"""
        parsed_tree, errors = self.parser.parse_input_stream(InputStream(scenario_content))
        self.assertEqual(errors, 0)
        model = self.parser.create_internal_model(parsed_tree, "test.osc", True)
        self.assertIsNotNone(model)
        param = model._ModelElement__children[1]
        values = param.get_resolved_value()
        self.assertEqual({"x": "override_x", "y": "value_y"}, values)

    @unittest.skip(reason="requires porting")
    def test_struct_with_keep_member_unknown(self):
        scenario_content = """
struct test_struct:
    x: string = "value_x"
    y: string = "value_y"
    
global foo: test_struct with:
        keep(it.UNKNOWN == "override_x")
"""
        parsed_tree, errors = self.parser.parse_input_stream(InputStream(scenario_content))
        self.assertEqual(errors, 0)
        model = self.parser.create_internal_model(parsed_tree, "test.osc", True)
        self.assertIsNotNone(model)
        param = model._ModelElement__children[1]
        values = param.get_resolved_value()  # parse should already fail!

    def test_parameter_to_dict_two_level(self):
        scenario_content = """
struct base_struct:
    x: string = "base_x"
    y: string = "base_y"
    
struct test_struct inherits base_struct:
    z: string = "value_z"
    
scenario test:
    foo: test_struct with:
        keep(it.x == "override_x")
"""
        parsed_tree, errors = self.parser.parse_input_stream(InputStream(scenario_content))
        self.assertEqual(errors, 0)
        model = self.parser.create_internal_model(parsed_tree, "test.osc", True)
        self.assertIsNotNone(model)
        param = model._ModelElement__children[2]._ModelElement__children[0]
        values = param.get_resolved_value()
        self.assertEqual({"x": "override_x", "y": "base_y", "z": "value_z"}, values)

    def test_parameter_struct_in_struct(self):
        scenario_content = """
struct base_struct:
    a: string = "base_a"
    b: string = "base_b"
    
struct test_struct:
    x: string = "value_x"
    y: base_struct
"""
        parsed_tree, errors = self.parser.parse_input_stream(InputStream(scenario_content))
        self.assertEqual(errors, 0)
        model = self.parser.create_internal_model(parsed_tree, "test.osc", True)
        self.assertIsNotNone(model)
        base_struct = model._ModelElement__children[0]

        values = base_struct.get_resolved_value()
        self.assertEqual({"a": "base_a", "b": "base_b"}, values)
        test_struct = model._ModelElement__children[1]
        values = test_struct.get_resolved_value()
        self.assertEqual({"x": "value_x", "y": {"a": "base_a", "b": "base_b"}}, values)

    def test_parameter_struct_in_struct_in_struct(self):
        scenario_content = """
struct base_struct:
    a: string = "base_a"
    
struct l1_struct:
    b: string = "value_x"
    c: base_struct
    
struct l2_struct:
    d: string = "value_x"
    e: l1_struct
"""
        parsed_tree, errors = self.parser.parse_input_stream(InputStream(scenario_content))
        self.assertEqual(errors, 0)
        model = self.parser.create_internal_model(parsed_tree, "test.osc", True)
        self.assertIsNotNone(model)
        base_struct = model._ModelElement__children[0]
        self.assertEqual({"a": "base_a"}, base_struct.get_resolved_value())
        l1_struct = model._ModelElement__children[1]
        self.assertEqual({'b': 'value_x', 'c': {'a': 'base_a'}}, l1_struct.get_resolved_value())
        l2_struct = model._ModelElement__children[2]
        self.assertEqual({'d': 'value_x', 'e': {'b': 'value_x', 'c': {
                         'a': 'base_a'}}}, l2_struct.get_resolved_value())

    def test_unknown_type(self):
        scenario_content = """
type length is SI(m: 1)
unit cm         of length is SI(m: 1, factor: 0.01)

global val1: UNKNOWN
"""
        parsed_tree, errors = self.parser.parse_input_stream(InputStream(scenario_content))
        self.assertEqual(errors, 0)
        model = self.parser.create_internal_model(parsed_tree, "test.osc")
        self.assertIsNone(model)

    def test_struct_param_default(self):
        scenario_content = """
struct test_struct:
    param1: string = "val1"
    param2: string = "val2"

global test_struct1: test_struct = test_struct(param2: 'OVERRIDE')
"""
        parsed_tree, errors = self.parser.parse_input_stream(InputStream(scenario_content))
        self.assertEqual(errors, 0)
        model = self.parser.create_internal_model(parsed_tree, "test.osc", True)
        self.assertIsNotNone(model)
        test_struct1 = model._ModelElement__children[1]
        self.assertEqual({'param1': 'val1', 'param2': 'OVERRIDE'},
                         test_struct1.get_resolved_value())

    def test_struct_param_function_wrong_type(self):
        scenario_content = """
struct other_type:
    param2: string = "val2"

struct test_struct:
    param1: string = "val1"
    param2: string = "val2"

global test_struct1: test_struct = other_type(param2: 'OVERRIDE')
"""
        parsed_tree, errors = self.parser.parse_input_stream(InputStream(scenario_content))
        self.assertEqual(errors, 0)
        model = self.parser.create_internal_model(parsed_tree, "test.osc", True)
        self.assertIsNone(model)

    def test_struct_assigned_empty(self):
        scenario_content = """
struct base_struct:
    base_param1: string = "base1"

struct test_struct:
    param_base_struct: base_struct = base_struct()
"""
        parsed_tree, errors = self.parser.parse_input_stream(InputStream(scenario_content))
        self.assertEqual(errors, 0)
        model = self.parser.create_internal_model(parsed_tree, "test.osc", True)
        self.assertIsNotNone(model)
        test_struct = model._ModelElement__children[1]
        self.assertEqual({'param_base_struct': {'base_param1': 'base1'}},
                         test_struct.get_resolved_value())

    def test_struct_assigned_with_params(self):
        scenario_content = """
struct base_struct:
    base_param1: string = "base1"

struct test_struct:
    param_base_struct: base_struct = base_struct('OVERRIDE')
"""
        parsed_tree, errors = self.parser.parse_input_stream(InputStream(scenario_content))
        self.assertEqual(errors, 0)
        model = self.parser.create_internal_model(parsed_tree, "test.osc", True)
        self.assertIsNotNone(model)
        test_struct = model._ModelElement__children[1]
        self.assertEqual({'param_base_struct': {'base_param1': 'OVERRIDE'}},
                         test_struct.get_resolved_value())

    def test_struct_assigned_by_var(self):
        scenario_content = """
struct base_struct:
    base_param1: string = "base1"

global test_struct1: base_struct = base_struct(base_param1: 'OVERRIDE')

struct test_struct:
    param_base_struct: base_struct = test_struct1
"""
        parsed_tree, errors = self.parser.parse_input_stream(InputStream(scenario_content))
        self.assertEqual(errors, 0)
        model = self.parser.create_internal_model(parsed_tree, "test.osc", True)
        self.assertIsNotNone(model)
        test_struct = model._ModelElement__children[2]
        self.assertEqual({'param_base_struct': {'base_param1': 'OVERRIDE'}},
                         test_struct.get_resolved_value())
