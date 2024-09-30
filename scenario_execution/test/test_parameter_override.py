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

import unittest

from scenario_execution.scenario_execution_base import ScenarioExecution
from scenario_execution.model.osc2_parser import OpenScenario2Parser
from scenario_execution.model.model_to_py_tree import create_py_tree
from .common import DebugLogger
from antlr4.InputStream import InputStream
import py_trees


class TestParameterOverride(unittest.TestCase):
    # pylint: disable=missing-function-docstring, protected-access, no-member, unused-variable, too-many-public-methods

    def setUp(self) -> None:
        self.logger = DebugLogger("")
        self.parser = OpenScenario2Parser(self.logger)
        self.tree = py_trees.composites.Sequence(name="", memory=True)

        self.scenario_execution = ScenarioExecution(debug=False, log_model=False, live_tree=False,
                                                    scenario_file='test.osc', output_dir="", logger=self.logger)
        self.tree = py_trees.composites.Sequence(name="", memory=True)

    def execute(self, scenario_content, override_parameters):
        parsed_tree = self.parser.parse_input_stream(InputStream(scenario_content))
        model = self.parser.create_internal_model(parsed_tree, self.tree, "test.osc", False,
                                                  scenario_parameter_overrides=override_parameters)
        self.tree = create_py_tree(model, self.tree, self.parser.logger, False)
        self.scenario_execution.tree = self.tree
        self.scenario_execution.run()

    def test_base_params_success(self):
        scenario_content = """
action log:
    msg: string

scenario test:
    test_string: string = "Test"
    test_float: float = 1.0
    test_bool: bool = true
    test_integer: int = 1
    do serial: 
        log(test_string)
        log(test_float)
        log(test_bool)
        log(test_integer)
"""
        override_parameters = {"test": {
            "test_string": "override",
            "test_bool": False,
            "test_float": 99.0,
            "test_integer": 42}}
        self.execute(scenario_content, override_parameters)
        self.assertEqual(self.logger.logs_info[1], "override")
        self.assertEqual(self.logger.logs_info[2], "99.0")
        self.assertEqual(self.logger.logs_info[3], "False")
        self.assertEqual(self.logger.logs_info[4], "42")

    def test_base_params_num_in_string(self):
        scenario_content = """
action log:
    msg: string

scenario test:
    test_string: string = "Test"
    do serial: 
        log(test_string)
"""
        override_parameters = {"test": {
            "test_string": 42.0}}
        self.execute(scenario_content, override_parameters)
        self.assertEqual(self.logger.logs_info[1], "42.0")

    def test_base_params_int_in_float(self):
        scenario_content = """
action log:
    msg: string

scenario test:
    test_float: float = 1.0
    do serial: 
        log(test_float)
"""
        override_parameters = {"test": {
            "test_float": 42}}
        self.execute(scenario_content, override_parameters)
        self.assertEqual(self.logger.logs_info[1], "42.0")

    def test_base_params_string_in_float(self):
        scenario_content = """
action log:
    msg: string

scenario test:
    test_float: float = 1.0
    do serial: 
        log(test_float)
"""
        override_parameters = {"test": {
            "test_float": "test"}}
        self.assertRaises(ValueError, self.execute, scenario_content, override_parameters)

    def test_base_params_string_in_bool(self):
        scenario_content = """
action log:
    msg: string

scenario test:
    test_bool: bool = true
    do serial: 
        log(test_bool)
"""
        override_parameters = {"test": {
            "test_bool": "test"}}
        self.assertRaises(ValueError, self.execute, scenario_content, override_parameters)

    def test_unknown_override(self):
        scenario_content = """
action log:
    msg: string

scenario test:
    test_bool: bool = true
    do serial: 
        log(test_bool)
"""
        override_parameters = {"test": {
            "UNKNOWN": "test"}}
        self.assertRaises(ValueError, self.execute, scenario_content, override_parameters)

    def test_physical_literal_base(self):
        scenario_content = """
import osc.helpers

scenario test:
    test_float: length = 1.0m
    do serial: 
        log(test_float)
"""
        override_parameters = {"test": {
            "test_float": 3.14}}
        self.execute(scenario_content, override_parameters)
        self.assertEqual(self.logger.logs_info[1], "3.14")

    def test_physical_literal(self):
        scenario_content = """
action log:
    msg: string

scenario test:
    test_float: length = 1.0m
    do serial: 
        log(test_float)
"""
        override_parameters = {"test": {
            "test_float": "bla"}}
        self.assertRaises(ValueError, self.execute, scenario_content, override_parameters)

    def test_struct_named_arg(self):
        scenario_content = """
action log:
    msg: string

struct base_struct:
    test_string: string = "test"

scenario test:
    my_struct: base_struct = base_struct(test_string: "override")
    do serial: 
        log(my_struct)
"""
        override_parameters = {"test": {
            "my_struct": {
                "test_string": "override"}}}
        self.execute(scenario_content, override_parameters)
        self.assertEqual(self.logger.logs_info[1], "{'test_string': 'override'}")

    def test_struct_pos_arg(self):
        scenario_content = """
action log:
    msg: string

struct base_struct:
    test_string: string = "test"

scenario test:
    my_struct: base_struct = base_struct("override")
    do serial: 
        log(my_struct)
"""
        override_parameters = {"test": {
            "my_struct": {
                "test_string": "override"}}}
        self.execute(scenario_content, override_parameters)
        self.assertEqual(self.logger.logs_info[1], "{'test_string': 'override'}")

    def test_struct_partial_override_1(self):
        scenario_content = """
action log:
    msg: string

struct base_struct:
    test_string: string = "test"
    test_float: float = 1.0

scenario test:
    my_struct: base_struct = base_struct(test_string: "override", test_float: 1.0)
    do serial: 
        log(my_struct)
"""
        override_parameters = {"test": {
            "my_struct": {
                "test_float": 42.0}}}
        self.execute(scenario_content, override_parameters)
        self.assertEqual(self.logger.logs_info[1], "{'test_string': 'override', 'test_float': 42.0}")

    def test_struct_partial_override_2(self):
        scenario_content = """
action log:
    msg: string

struct base_struct:
    test_string: string = "test"
    test_float: float = 1.0

scenario test:
    my_struct: base_struct = base_struct(test_string: "override", test_float: 42.0)
    do serial: 
        log(my_struct)
"""
        override_parameters = {"test": {
            "my_struct": {
                "test_string": 'hello'}}}
        self.execute(scenario_content, override_parameters)
        self.assertEqual(self.logger.logs_info[1], "{'test_string': 'hello', 'test_float': 42.0}")

    def test_struct_no_override(self):
        scenario_content = """
action log:
    msg: string

struct base_struct:
    test_string: string = "test"
    test_float: float = 1.0

scenario test:
    my_struct: base_struct = base_struct(test_string: "override", test_float: 42.0)
    do serial: 
        log(my_struct)
"""
        override_parameters = {"test": {
            "my_struct": {}}}
        self.execute(scenario_content, override_parameters)
        self.assertEqual(self.logger.logs_info[1], "{'test_string': 'override', 'test_float': 42.0}")

    def test_struct_no_init_overload_1(self):
        scenario_content = """
action log:
    msg: string

struct base_struct:
    test_string: string = "test"
    test_float: float = 1.0

scenario test:
    my_struct: base_struct = base_struct()
    do serial: 
        log(my_struct)
"""
        override_parameters = {"test": {
            "my_struct": {
                "test_string": "override"
            }}}
        self.execute(scenario_content, override_parameters)
        self.assertEqual(self.logger.logs_info[1], "{'test_string': 'override', 'test_float': 1.0}")

    def test_struct_no_init_overload_2(self):
        scenario_content = """
action log:
    msg: string

struct base_struct:
    test_string: string = "test"
    test_float: float = 1.0

scenario test:
    my_struct: base_struct = base_struct()
    do serial: 
        log(my_struct)
"""
        override_parameters = {"test": {
            "my_struct": {
                "test_float": 42.0
            }}}
        self.execute(scenario_content, override_parameters)
        self.assertEqual(self.logger.logs_info[1], "{'test_string': 'test', 'test_float': 42.0}")

    def test_struct_no_init_overload_all(self):
        scenario_content = """
action log:
    msg: string

struct base_struct:
    test_string: string = "test"
    test_float: float = 1.0

scenario test:
    my_struct: base_struct = base_struct()
    do serial: 
        log(my_struct)
"""
        override_parameters = {"test": {
            "my_struct": {
                "test_string": "override",
                "test_float": 42.0
            }}}
        self.execute(scenario_content, override_parameters)
        self.assertEqual(self.logger.logs_info[1], "{'test_string': 'override', 'test_float': 42.0}")

    def test_struct_sub_struct_member(self):
        scenario_content = """
action log:
    msg: string

struct sub_struct:
    test_sub_string: string = "bla"
    
struct base_struct:
    test_sub_struct: sub_struct

scenario test:
    my_struct: base_struct = base_struct(test_sub_struct: sub_struct(test_sub_string: 'test'))
    do serial: 
        log(my_struct)
"""
        override_parameters = {"test": {
            "my_struct": {
                "test_sub_struct": {
                    "test_sub_string": "override"
                }
            }}}
        self.execute(scenario_content, override_parameters)
        self.assertEqual(self.logger.logs_info[1], "{'test_sub_struct': {'test_sub_string': 'override'}}")

    def test_struct_no_init_overload_sub_struct_member(self):
        scenario_content = """
action log:
    msg: string

struct sub_struct:
    test_sub_string: string = "bla"
    
struct base_struct:
    test_string: string = "test"
    test_float: float = 1.0
    test_sub_struct: sub_struct

scenario test:
    my_struct: base_struct = base_struct()
    do serial: 
        log(my_struct)
"""
        override_parameters = {"test": {
            "my_struct": {
                "test_float": 42.0,
                "test_sub_struct": {
                    "test_sub_string": "override"
                }
            }}}
        self.execute(scenario_content, override_parameters)
        self.assertEqual(
            self.logger.logs_info[1], "{'test_string': 'test', 'test_float': 42.0, 'test_sub_struct': {'test_sub_string': 'override'}}")

    def test_struct_no_init_overload_sub_struct_member_wrong_type(self):
        scenario_content = """
action log:
    msg: string

struct sub_struct:
    test_sub_float: float = 0.0
    
struct base_struct:
    test_sub_struct: sub_struct

scenario test:
    my_struct: base_struct = base_struct()
    do serial: 
        log(my_struct)
"""
        override_parameters = {"test": {
            "my_struct": {
                "test_sub_struct": {
                    "test_sub_float": "override"
                }
            }}}
        self.assertRaises(ValueError, self.execute, scenario_content, override_parameters)

    def test_struct_no_init_overload_sub_struct_member_unknown(self):
        scenario_content = """
action log:
    msg: string

struct sub_struct:
    test_sub_float: float = 0.0
    
struct base_struct:
    test_sub_struct: sub_struct

scenario test:
    my_struct: base_struct = base_struct()
    do serial: 
        log(my_struct)
"""
        override_parameters = {"test": {
            "my_struct": {
                "test_sub_struct": {
                    "UNKNOWN": "override"
                }
            }}}
        self.assertRaises(ValueError, self.execute, scenario_content, override_parameters)

    def test_struct_no_init_overload_sub_sub_struct_member(self):
        scenario_content = """
action log:
    msg: string

struct sub_sub_struct:
    test_sub_sub_float: float = 0.0
    
struct sub_struct:
    test_sub_sub_struct: sub_sub_struct
    
struct base_struct:
    test_sub_struct: sub_struct

scenario test:
    my_struct: base_struct = base_struct()
    do serial: 
        log(my_struct)
"""
        override_parameters = {"test": {
            "my_struct": {
                "test_sub_struct": {
                    "test_sub_sub_struct": {
                        "test_sub_sub_float": 42.0
                    }
                }
            }}}
        self.execute(scenario_content, override_parameters)
        self.assertEqual(self.logger.logs_info[1], "{'test_sub_struct': {'test_sub_sub_struct': {'test_sub_sub_float': 42.0}}}")

    def test_struct_no_init_overload_sub_sub_struct_member_unknown(self):
        scenario_content = """
action log:
    msg: string

struct sub_sub_struct:
    test_sub_sub_float: float = 0.0
    
struct sub_struct:
    test_sub_sub_struct: sub_sub_struct
    
struct base_struct:
    test_sub_struct: sub_struct

scenario test:
    my_struct: base_struct = base_struct()
    do serial: 
        log(my_struct)
"""
        override_parameters = {"test": {
            "my_struct": {
                "test_sub_struct": {
                    "test_sub_sub_struct": {
                        "UNKNOWN": 42.0
                    }
                }
            }}}
        self.assertRaises(ValueError, self.execute, scenario_content, override_parameters)

    def test_struct_no_init_overload_sub_sub_struct_member_physical_literal_test(self):
        scenario_content = """
import osc.helpers

struct sub_sub_struct:
    test_sub_sub_float: length = 0.0m
    
struct sub_struct:
    test_sub_sub_struct: sub_sub_struct
    
struct base_struct:
    test_sub_struct: sub_struct

scenario test:
    my_struct: base_struct = base_struct()
    do serial: 
        log(my_struct)
"""
        override_parameters = {"test": {
            "my_struct": {
                "test_sub_struct": {
                    "test_sub_sub_struct": {
                        "test_sub_sub_float": 42.0
                    }
                }
            }}}
        self.execute(scenario_content, override_parameters)
        self.assertEqual(self.logger.logs_info[1], "{'test_sub_struct': {'test_sub_sub_struct': {'test_sub_sub_float': 42.0}}}")

    def test_struct_no_init_overload_sub_sub_struct_member_physical_literal_invalid(self):
        scenario_content = """
action log:
    msg: string

struct sub_sub_struct:
    test_sub_sub_float: length = 0.0m
    
struct sub_struct:
    test_sub_sub_struct: sub_sub_struct
    
struct base_struct:
    test_sub_struct: sub_struct

scenario test:
    my_struct: base_struct = base_struct()
    do serial: 
        log(my_struct)
"""
        override_parameters = {"test": {
            "my_struct": {
                "test_sub_struct": {
                    "test_sub_sub_struct": {
                        "test_sub_sub_float": "INVALID"
                    }
                }
            }}}
        self.assertRaises(ValueError, self.execute, scenario_content, override_parameters)

    def test_struct_no_init_overload_sub_sub_struct_member_physical_literal(self):
        scenario_content = """
action log:
    msg: string

scenario test:
    my_base_val: string = "foo"
    my_derived_val: string = my_base_val
    do serial: 
        log(my_derived_val)
"""
        override_parameters = {"test": {
            "my_derived_val": "override"}}
        self.assertRaises(ValueError, self.execute, scenario_content, override_parameters)

    def test_pose3d_example(self):
        scenario_content = """
import osc.helpers

scenario test:
    my_goal: pose_3d
    do serial: 
        log(my_goal)
"""
        override_parameters = {"test": {
            "my_goal": {
                "orientation": {
                    "pitch": 1.23
                }}
        }}
        self.execute(scenario_content, override_parameters)
        self.assertEqual(
            self.logger.logs_info[1], "{'position': {'x': 0.0, 'y': 0.0, 'z': 0.0}, 'orientation': {'roll': 0.0, 'pitch': 1.23, 'yaw': 0.0}}")

    def test_string_empty(self):
        scenario_content = """
action log:
    msg: string

scenario test:
    my_val: string
    do serial: 
        log(my_val)
"""
        override_parameters = {"test": {
            "my_val": "override"
        }}
        self.execute(scenario_content, override_parameters)
        self.assertEqual(self.logger.logs_info[1], "override")

    def test_int_empty(self):
        scenario_content = """
action log:
    msg: string

scenario test:
    my_val: int
    do serial: 
        log(my_val)
"""
        override_parameters = {"test": {
            "my_val": 42
        }}
        self.execute(scenario_content, override_parameters)
        self.assertEqual(self.logger.logs_info[1], "42")

    def test_int_empty_wrong_override(self):
        scenario_content = """
action log:
    msg: string

scenario test:
    my_val: int
    do serial: 
        log(my_val)
"""
        override_parameters = {"test": {
            "my_val": 42.0
        }}
        self.assertRaises(ValueError, self.execute, scenario_content, override_parameters)

    def test_float_empty(self):
        scenario_content = """
action log:
    msg: string

scenario test:
    my_val: float
    do serial: 
        log(my_val)
"""
        override_parameters = {"test": {
            "my_val": 42.0
        }}
        self.execute(scenario_content, override_parameters)
        self.assertEqual(self.logger.logs_info[1], "42.0")

    def test_float_empty_int_override(self):
        scenario_content = """
action log:
    msg: string

scenario test:
    my_val: float
    do serial: 
        log(my_val)
"""
        override_parameters = {"test": {
            "my_val": 42
        }}
        self.execute(scenario_content, override_parameters)
        self.assertEqual(self.logger.logs_info[1], "42.0")

    def test_float_empty_wrong_override(self):
        scenario_content = """
action log:
    msg: string

scenario test:
    my_val: float
    do serial: 
        log(my_val)
"""
        override_parameters = {"test": {
            "my_val": "invalid"
        }}
        self.assertRaises(ValueError, self.execute, scenario_content, override_parameters)

    def test_bool_empty(self):
        scenario_content = """
action log:
    msg: string

scenario test:
    my_val: bool
    do serial: 
        log(my_val)
"""
        override_parameters = {"test": {
            "my_val": True
        }}
        self.execute(scenario_content, override_parameters)
        self.assertEqual(self.logger.logs_info[1], "True")

    def test_bool_empty_wrong_override(self):
        scenario_content = """
action log:
    msg: string

scenario test:
    my_val: bool
    do serial: 
        log(my_val)
"""
        override_parameters = {"test": {
            "my_val": "invalid"
        }}
        self.assertRaises(ValueError, self.execute, scenario_content, override_parameters)

    def test_physical_literal_empty(self):
        scenario_content = """
import osc.helpers

scenario test:
    my_val: length
    do serial: 
        log(my_val)
"""
        override_parameters = {"test": {
            "my_val": 42.0
        }}
        self.execute(scenario_content, override_parameters)
        self.assertEqual(self.logger.logs_info[1], "42.0")

    def test_physical_literal_empty_invalid_override_test(self):
        scenario_content = """
import osc.helpers

scenario test:
    my_val: length
    do serial: 
        log(my_val)
"""
        override_parameters = {"test": {
            "my_val": "invalid"
        }}
        self.assertRaises(ValueError, self.execute, scenario_content, override_parameters)

    def test_physical_literal_empty_override(self):
        scenario_content = """
action log:
    msg: string
    
scenario test:
    var my_val: float
    do serial: 
        log(my_val)
"""
        override_parameters = {"test": {
            "my_val": 3.14
        }}
        self.assertRaises(ValueError, self.execute, scenario_content, override_parameters)

    def test_physical_literal_empty_invalid_override(self):
        scenario_content = """
action log:
    msg: string

actor bla
 
scenario test:
    my_val: bla
    do serial: 
        log(my_val)
"""
        override_parameters = {"test": {
            "my_val": 3.14
        }}
        self.assertRaises(ValueError, self.execute, scenario_content, override_parameters)

    def test_struct_funct_app_no_val(self):
        scenario_content = """
action log:
    msg: string
    
struct base_struct:
    test_float: float = 0.0
 
scenario test:
    my_val: base_struct = base_struct()
    do serial: 
        log(my_val)
"""
        override_parameters = {"test": {
            "my_val": {
                "test_float": 4.2
            }
        }}
        self.execute(scenario_content, override_parameters)
        self.assertEqual(self.logger.logs_info[1], "{'test_float': 4.2}")

    def test_list_struct(self):
        scenario_content = """
action log:
    msg: string

struct test_struct:
    test_int: int = 0
    test_float: float = 0.0
 
scenario test:
    my_val: list of test_struct = [test_struct(1, 1.0)]
    do serial: 
        log(my_val)
"""
        override_parameters = {"test": {
            "my_val": [
                {"test_int": 3, "test_float": 3.22},
                {"test_int": 4, "test_float": 4.22}
            ]
        }}
        self.execute(scenario_content, override_parameters)
        self.assertEqual(self.logger.logs_info[1], "[{'test_int': 3, 'test_float': 3.22}, {'test_int': 4, 'test_float': 4.22}]")

    def test_list_struct_invalid_override(self):
        scenario_content = """
action log:
    msg: string

struct test_struct:
    test_int: int = 0
    test_float: float = 0.0
 
scenario test:
    my_val: list of test_struct = [test_struct(1, 1.0)]
    do serial: 
        log(my_val)
"""
        override_parameters = {"test": {
            "my_val": [
                {"test_int": 3, "test_float": 3.22},
                {"test_int": 4, "UNKNOWN": 4.22}
            ]
        }}
        self.assertRaises(ValueError, self.execute, scenario_content, override_parameters)

    def test_list_struct_empty(self):
        scenario_content = """
action log:
    msg: string

struct test_struct:
    test_int: int = 0
    test_float: float = 0.0
 
scenario test:
    my_val: list of test_struct
    do serial: 
        log(my_val)
"""
        override_parameters = {"test": {
            "my_val": [
                {"test_int": 3, "test_float": 3.22},
                {"test_int": 4, "test_float": 4.22}
            ]
        }}
        self.execute(scenario_content, override_parameters)
        self.assertEqual(self.logger.logs_info[1], "[{'test_int': 3, 'test_float': 3.22}, {'test_int': 4, 'test_float': 4.22}]")

    def test_list_base(self):
        scenario_content = """
action log:
    msg: string
 
scenario test:
    my_val: list of float = [ 1.0, 1.1, 1.2 ]
    do serial: 
        log(my_val)
"""
        override_parameters = {"test": {
            "my_val": [4.0, 4.1]
        }}
        self.execute(scenario_content, override_parameters)
        self.assertEqual(self.logger.logs_info[1], "[4.0, 4.1]")

    def test_list_base_empty(self):
        scenario_content = """
action log:
    msg: string
 
scenario test:
    my_val: list of float
    do serial: 
        log(my_val)
"""
        override_parameters = {"test": {
            "my_val": [4.0, 4.1]
        }}
        self.execute(scenario_content, override_parameters)
        self.assertEqual(self.logger.logs_info[1], "[4.0, 4.1]")

    def test_list_base_empty_invalid_sub_entry(self):
        scenario_content = """
action log:
    msg: string
 
scenario test:
    my_val: list of float
    do serial: 
        log(my_val)
"""
        override_parameters = {"test": {
            "my_val": [4.0, 'bla']
        }}
        self.assertRaises(ValueError, self.execute, scenario_content, override_parameters)

    def test_list_base_empty_invalid_override(self):
        scenario_content = """
action log:
    msg: string
 
scenario test:
    my_val: list of float
    do serial: 
        log(my_val)
"""
        override_parameters = {"test": {
            "my_val": {'invalid': 'val'}
        }}
        self.assertRaises(ValueError, self.execute, scenario_content, override_parameters)

    def test_list_struct_list_param_empty(self):
        scenario_content = """
action log:
    msg: string

struct test_struct:
    test_int: int = 0
    test_list: list of float
 
scenario test:
    my_val: test_struct = test_struct(9, [ 9.1, 9.2, 9.3])
    do serial: 
        log(my_val)
"""
        override_parameters = {"test": {
            "my_val": {
                "test_int": 42,
                "test_list": [4.0, 4.1]
            }
        }}
        self.execute(scenario_content, override_parameters)
        self.assertEqual(self.logger.logs_info[1], "{'test_int': 42, 'test_list': [4.0, 4.1]}")

    def test_list_struct_list_param_set(self):
        scenario_content = """
action log:
    msg: string

struct test_struct:
    test_int: int = 0
    test_list: list of float = [0.0]
 
scenario test:
    my_val: test_struct = test_struct()
    do serial: 
        log(my_val)
"""
        override_parameters = {"test": {
            "my_val": {
                "test_int": 42,
                "test_list": [4.0, 4.1]
            }
        }}
        self.execute(scenario_content, override_parameters)
        self.assertEqual(self.logger.logs_info[1], "{'test_int': 42, 'test_list': [4.0, 4.1]}")

    def test_list_struct_list_param_unset(self):
        scenario_content = """
action log:
    msg: string

struct test_struct:
    test_int: int = 0
    test_list: list of float
 
scenario test:
    my_val: test_struct = test_struct()
    do serial: 
        log(my_val)
"""
        override_parameters = {"test": {
            "my_val": {
                "test_int": 42,
                "test_list": [4.0, 4.1]
            }
        }}
        self.execute(scenario_content, override_parameters)
        self.assertEqual(self.logger.logs_info[1], "{'test_int': 42, 'test_list': [4.0, 4.1]}")

    def test_list_struct_empty_test(self):
        scenario_content = """
action log:
    msg: string

struct test_struct:
    test_int: int = 0
    test_list: list of float = [0.0]
 
scenario test:
    my_val: test_struct
    do serial: 
        log(my_val)
"""
        override_parameters = {"test": {
            "my_val": {
                "test_int": 42,
                "test_list": [4.0, 4.1]
            }
        }}
        self.execute(scenario_content, override_parameters)
        self.assertEqual(self.logger.logs_info[1], "{'test_int': 42, 'test_list': [4.0, 4.1]}")

    def test_list_struct_with_sub_struct_list_no_init(self):
        scenario_content = """
action log:
    msg: string

struct sub_struct:
    test_sub_list: list of string
    
struct test_struct:
    test_list: list of sub_struct
 
scenario test:
    my_val: test_struct
    do serial: 
        log(my_val)
"""
        override_parameters = {"test": {
            "my_val": {
                "test_list": [{
                    "test_sub_list": ["foo", "bar"]
                }, {
                    "test_sub_list": ["Hello", "World"]
                }]
            }
        }}
        self.execute(scenario_content, override_parameters)
        self.assertEqual(self.logger.logs_info[1],
                         "{'test_list': [{'test_sub_list': ['foo', 'bar']}, {'test_sub_list': ['Hello', 'World']}]}")

    def test_list_struct_with_sub_struct_list_init_sub_struct(self):
        scenario_content = """
action log:
    msg: string

struct sub_struct:
    test_sub_list: list of string = ["foo"]
    
struct test_struct:
    test_list: list of sub_struct
 
scenario test:
    my_val: test_struct
    do serial: 
        log(my_val)
"""
        override_parameters = {"test": {
            "my_val": {
                "test_list": [{
                    "test_sub_list": ["foo", "bar"]
                }, {
                    "test_sub_list": ["Hello", "World"]
                }]
            }
        }}
        self.execute(scenario_content, override_parameters)
        self.assertEqual(self.logger.logs_info[1],
                         "{'test_list': [{'test_sub_list': ['foo', 'bar']}, {'test_sub_list': ['Hello', 'World']}]}")

    def test_list_struct_with_sub_struct_list_init_struct(self):
        scenario_content = """
action log:
    msg: string

struct sub_struct:
    test_sub_list: list of string
    
struct test_struct:
    test_list: list of sub_struct = [ sub_struct("val")]
 
scenario test:
    my_val: test_struct
    do serial: 
        log(my_val)
"""
        override_parameters = {"test": {
            "my_val": {
                "test_list": [{
                    "test_sub_list": ["foo", "bar"]
                }, {
                    "test_sub_list": ["Hello", "World"]
                }]
            }
        }}
        self.execute(scenario_content, override_parameters)
        self.assertEqual(self.logger.logs_info[1],
                         "{'test_list': [{'test_sub_list': ['foo', 'bar']}, {'test_sub_list': ['Hello', 'World']}]}")

    def test_list_struct_param_init(self):
        scenario_content = """
action log:
    msg: string

struct test_struct:
    test_list: list of float = [0.0]
 
scenario test:
    my_val: list of test_struct = [ test_struct() ]
    do serial: 
        log(my_val)
"""
        override_parameters = {"test": {
            "my_val": [
                {"test_list": [4.0, 4.1]},
                {"test_list": [5.0, 5.1]}
            ]
        }}
        self.execute(scenario_content, override_parameters)
        self.assertEqual(self.logger.logs_info[1], "[{'test_list': [4.0, 4.1]}, {'test_list': [5.0, 5.1]}]")

    def test_list_struct_param_no_init(self):
        scenario_content = """
action log:
    msg: string

struct test_struct:
    test_list: list of float = [0.0]
 
scenario test:
    my_val: list of test_struct
    do serial: 
        log(my_val)
"""
        override_parameters = {"test": {
            "my_val": [
                {"test_list": [4.0, 4.1]},
                {"test_list": [5.0, 5.1]}
            ]
        }}
        self.execute(scenario_content, override_parameters)
        self.assertEqual(self.logger.logs_info[1], "[{'test_list': [4.0, 4.1]}, {'test_list': [5.0, 5.1]}]")

    def test_list_struct_param_full_init(self):
        scenario_content = """
action log:
    msg: string

struct test_struct:
    test_list: list of float
 
scenario test:
    my_val: list of test_struct = [ test_struct([42.0, 42.1])]
    do serial: 
        log(my_val)
"""
        override_parameters = {"test": {
            "my_val": [
                {"test_list": [4.0, 4.1]},
                {"test_list": [5.0, 5.1]}
            ]
        }}
        self.execute(scenario_content, override_parameters)
        self.assertEqual(self.logger.logs_info[1], "[{'test_list': [4.0, 4.1]}, {'test_list': [5.0, 5.1]}]")

    def test_list_struct_param_clear(self):
        scenario_content = """
action log:
    msg: string

struct test_struct:
    test_list: list of float
 
scenario test:
    my_val: list of test_struct = [ test_struct([42.0, 42.1])]
    do serial: 
        log(my_val)
"""
        override_parameters = {"test": {
            "my_val": []
        }}
        self.execute(scenario_content, override_parameters)
        self.assertEqual(self.logger.logs_info[1], "[]")

    def test_list_struct_param_length(self):
        scenario_content = """
import osc.helpers

struct test_struct:
    test_list: list of length
 
scenario test:
    my_val: list of test_struct = [ test_struct([2.0m, 2.1m])]
    do serial: 
        log(my_val)
"""
        override_parameters = {"test": {
            "my_val": [
                {"test_list": [14.0, 14.1]},
                {"test_list": [15.0, 15.1]}
            ]
        }}
        self.execute(scenario_content, override_parameters)
        self.assertEqual(self.logger.logs_info[1], "[{'test_list': [14.0, 14.1]}, {'test_list': [15.0, 15.1]}]")
