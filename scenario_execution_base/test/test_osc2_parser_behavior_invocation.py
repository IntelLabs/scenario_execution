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
Test for osc2_parser
"""
import unittest

from scenario_execution_base.model.osc2_parser import OpenScenario2Parser
from scenario_execution_base.utils.logging import Logger
from antlr4.InputStream import InputStream


class TestOSC2Parser(unittest.TestCase):
    # pylint: disable=missing-function-docstring, protected-access, no-member, unused-variable
    """
    Unit test for osc2_parser
    """

    def setUp(self) -> None:
        self.parser = OpenScenario2Parser(Logger('test'))

    def test_behavior_invocation_param_override(self):
        scenario_content = """
actor amr_object:
    name: string = 'amr'
    model_file: string = 'amr'

struct pose_3d:
    x: string = 'x_val'

action amr_object.spawn:
    spawn_pose: pose_3d
    world_name: string = 'default'
    namespace: string = ''
    entity_name: string = ''
    model_file: string = ''
    xacro_arguments: string = ''        # comma-separated list of argument key:=value pairs

scenario test:
    test_obstacle1: amr_object with:
        keep(it.name == 'test_obstacle1')
        keep(it.model_file == 'test_model')
    do parallel:
        test_drive: serial:
            spawn_it: test_obstacle1.spawn() with:
                keep(it.namespace == 'bll')
"""
        parsed_tree, errors = self.parser.parse_input_stream(InputStream(scenario_content))
        self.assertEqual(errors, 0)
        model = self.parser.create_internal_model(parsed_tree, "test.osc", True)
        self.assertIsNotNone(model)
        behavior = model._ModelElement__children[3]._ModelElement__children[1]._ModelElement__children[0]._ModelElement__children[0]._ModelElement__children[0]
        params = behavior.get_resolved_value()
        self.assertEqual({'spawn_pose': {'x': 'x_val'}, 'world_name': 'default',
                         'namespace': 'bll', 'entity_name': '', 'model_file': '', 'xacro_arguments': ''}, params)

    def test_behavior_named_arg(self):
        scenario_content = """
actor amr_object

action amr_object.spawn:
    param1: string = 'val1'
    param2: string = 'val2'

scenario test:
    test_obstacle1: amr_object
    do parallel:
        spawn_it: test_obstacle1.spawn(param1: 'override1')
"""
        parsed_tree, errors = self.parser.parse_input_stream(InputStream(scenario_content))
        self.assertEqual(errors, 0)
        model = self.parser.create_internal_model(parsed_tree, "test.osc", True)
        self.assertIsNotNone(model)

        behavior = model._ModelElement__children[2]._ModelElement__children[1]._ModelElement__children[0]._ModelElement__children[0]
        params = behavior.get_resolved_value()
        self.assertEqual(params, {'param1': 'override1', 'param2': 'val2'})

    def test_behavior_from_base_actor(self):
        scenario_content = """
actor amr_object

action amr_object.spawn:
    param1: string

actor robot inherits amr_object

scenario test:
    test_obstacle1: robot
    do parallel:
        spawn_it: test_obstacle1.spawn(param1: 'override1')
"""
        parsed_tree, errors = self.parser.parse_input_stream(InputStream(scenario_content))
        self.assertEqual(errors, 0)
        model = self.parser.create_internal_model(parsed_tree, "test.osc", True)
        self.assertIsNotNone(model)

    def test_behavior_invalid_param_name(self):
        scenario_content = """
actor amr_object

action amr_object.spawn:
    name: string = 'NOTALLOWED'

scenario test:
    test_obstacle1: amr_object
    do parallel:
        spawn_it: test_obstacle1.spawn(param1: 'override1')
"""
        parsed_tree, errors = self.parser.parse_input_stream(InputStream(scenario_content))
        self.assertEqual(errors, 0)
        model = self.parser.create_internal_model(parsed_tree, "test.osc")
        self.assertIsNone(model)

    def test_behavior_invalid_param_associated_actor(self):
        scenario_content = """
actor amr_object

action amr_object.spawn:
    associated_actor: string = 'NOTALLOWED'
    param1: string

scenario test:
    test_obstacle1: amr_object
    do parallel:
        spawn_it: test_obstacle1.spawn(param1: 'override1')
"""
        parsed_tree, errors = self.parser.parse_input_stream(InputStream(scenario_content))
        self.assertEqual(errors, 0)
        model = self.parser.create_internal_model(parsed_tree, "test.osc", True)
        self.assertIsNone(model)

    def test_behavior_positional_arg(self):
        scenario_content = """
action test:
    param1: string = 'base'

scenario test:
    do parallel:
        spawn_it: test('override1')
"""
        parsed_tree, errors = self.parser.parse_input_stream(InputStream(scenario_content))
        self.assertEqual(errors, 0)
        model = self.parser.create_internal_model(parsed_tree, "test.osc", True)
        self.assertIsNotNone(model)

        behavior = model._ModelElement__children[1]._ModelElement__children[0]._ModelElement__children[0]._ModelElement__children[0]
        params = behavior.get_resolved_value()
        self.assertEqual(params, {'param1': 'override1'})

    @unittest.skip(reason="requires porting")
    def test_behavior_empty_args(self):
        scenario_content = """
action test:
    param1: string
    param2: string

scenario test:
    do parallel:
        spawn_it: test()
"""
        parsed_tree, errors = self.parser.parse_input_stream(InputStream(scenario_content))
        self.assertEqual(errors, 0)
        model = self.parser.create_internal_model(parsed_tree, "test.osc", True)
        self.assertIsNone(model)

    @unittest.skip(reason="requires porting")
    def test_behavior_partially_defined_args(self):
        scenario_content = """
action test:
    param1: string = 'val1'
    param2: string

scenario test:
    do parallel:
        spawn_it: test()
"""
        parsed_tree, errors = self.parser.parse_input_stream(InputStream(scenario_content))
        self.assertEqual(errors, 0)
        model = self.parser.create_internal_model(parsed_tree, "test.osc")
        self.assertIsNone(model)

    @unittest.skip(reason="requires porting")
    def test_behavior_partially_overridden_positional_args(self):
        scenario_content = """
action test:
    param1: string
    param2: string

scenario test:
    do parallel:
        spawn_it: test('override1')
"""
        parsed_tree, errors = self.parser.parse_input_stream(InputStream(scenario_content))
        self.assertEqual(errors, 0)
        model = self.parser.create_internal_model(parsed_tree, "test.osc")
        self.assertIsNone(model)

    @unittest.skip(reason="requires porting")
    def test_behavior_partially_overridden_named_args(self):
        scenario_content = """
action test:
    param1: string
    param2: string

scenario test:
    do parallel:
        spawn_it: test(param2: 'override2')
"""
        parsed_tree, errors = self.parser.parse_input_stream(InputStream(scenario_content))
        self.assertEqual(errors, 0)
        model = self.parser.create_internal_model(parsed_tree, "test.osc")
        self.assertIsNone(model)

    def test_behavior_too_many_named_args(self):
        scenario_content = """
action test:
    param1: string
    param2: string

scenario test:
    do parallel:
        spawn_it: test(param1: 'override1', param2: 'override2', param3: 'override3')
"""
        parsed_tree, errors = self.parser.parse_input_stream(InputStream(scenario_content))
        self.assertEqual(errors, 0)
        model = self.parser.create_internal_model(parsed_tree, "test.osc")
        self.assertIsNone(model)

    def test_behavior_too_many_positional_args(self):
        scenario_content = """
action test:
    param1: string
    param2: string

scenario test:
    do parallel:
        spawn_it: test('override1', 'override2', 'override3')
"""
        parsed_tree, errors = self.parser.parse_input_stream(InputStream(scenario_content))
        self.assertEqual(errors, 0)
        model = self.parser.create_internal_model(parsed_tree, "test.osc")
        self.assertIsNone(model)

    def test_behavior_unknown_named_args(self):
        scenario_content = """
action test:
    param1: string
    param2: string

scenario test:
    do parallel:
        spawn_it: test(param1: 'override1', UNKNOWN: 'override2')
"""
        parsed_tree, errors = self.parser.parse_input_stream(InputStream(scenario_content))
        self.assertEqual(errors, 0)
        model = self.parser.create_internal_model(parsed_tree, "test.osc")
        self.assertIsNone(model)

    @unittest.skip(reason="requires porting")
    def test_behavior_struct_param_named_override(self):
        scenario_content = """
struct test_struct:
    param1: string = "val1"
    param2: string = "val2"

action test_action:
    struct_param: test_struct

scenario test:
    do serial:
        test_action() with:
            keep(it.struct_param == test_struct(param2: 'OVERRIDE'))
"""
        parsed_tree, errors = self.parser.parse_input_stream(InputStream(scenario_content))
        self.assertEqual(errors, 0)
        model = self.parser.create_internal_model(parsed_tree, "test.osc", True)
        self.assertIsNotNone(model)

        behavior = model._ModelElement__children[2]._ModelElement__children[1]._ModelElement__children[0]._ModelElement__children[1]
        params = behavior.get_resolved_value()
        self.assertEqual(params, {'struct_param': {'param1': 'val1', 'param2': 'OVERRIDE'}})

    def test_behavior_struct_param_positional_override(self):
        scenario_content = """
struct test_struct:
    param1: string = "val1"
    param2: string = "val2"

action test_action:
    struct_param: test_struct

scenario test:
    do serial:
        test_action() with:
            keep(it.struct_param == test_struct('OVERRIDE'))
"""
        parsed_tree, errors = self.parser.parse_input_stream(InputStream(scenario_content))
        self.assertEqual(errors, 0)
        model = self.parser.create_internal_model(parsed_tree, "test.osc", True)
        self.assertIsNotNone(model)

        behavior = model._ModelElement__children[2]._ModelElement__children[0]._ModelElement__children[0]._ModelElement__children[0]
        params = behavior.get_resolved_value()
        self.assertEqual(params, {'struct_param': {'param1': 'OVERRIDE', 'param2': 'val2'}})

    def test_behavior_struct_param_partially_named_positional_override(self):
        scenario_content = """
struct test_struct:
    param1: string = "val1"
    param2: string = "val2"
    param3: string = "val3"

action test_action:
    struct_param: test_struct

scenario test:
    do serial:
        test_action() with:
            keep(it.struct_param == test_struct('OVERRIDE1', param3: 'OVERRIDE3'))
"""
        parsed_tree, errors = self.parser.parse_input_stream(InputStream(scenario_content))
        self.assertEqual(errors, 0)
        model = self.parser.create_internal_model(parsed_tree, "test.osc", True)
        self.assertIsNotNone(model)

        behavior = model._ModelElement__children[2]._ModelElement__children[0]._ModelElement__children[0]._ModelElement__children[0]
        params = behavior.get_resolved_value()
        self.assertEqual(params, {'struct_param': {'param1': 'OVERRIDE1',
                         'param2': 'val2', 'param3': 'OVERRIDE3'}})

    @unittest.skip(reason="requires porting")
    def test_behavior_struct_param_spec_incomplete(self):
        scenario_content = """
struct test_struct:
    param1: string = "val1"
    param2: string

action test_action:
    struct_param: test_struct

scenario test:
    do serial:
        test_action() with:
            keep(it.struct_param == test_struct('OVERRIDE1'))
"""
        parsed_tree, errors = self.parser.parse_input_stream(InputStream(scenario_content))
        self.assertEqual(errors, 0)
        model = self.parser.create_internal_model(parsed_tree, "test.osc")
        self.assertIsNone(model)

    def test_base_vals(self):
        scenario_content = """
type length is SI(m: 1)
unit m          of length is SI(m: 1, factor: 1)
type angle is SI(rad: 1)
unit rad    of angle is SI(rad: 1, factor: 1)

actor amr_object:
    model: string

struct position_3d:
    x: length = 0m
    y: length = 0m
    z: length = 0m

struct orientation_3d:
    roll: angle = 0rad
    pitch: angle = 0rad
    yaw: angle = 0rad

struct pose_3d:
    position: position_3d
    orientation: orientation_3d

action amr_object.spawn:
    spawn_pose: pose_3d

actor differential_drive_robot inherits amr_object:
    namespace: string = ''

scenario nav2_simulation_nav_to_pose:
    turtlebot4: differential_drive_robot with:
        keep(it.model == 'topic:///robot_description')
    do parallel:
            turtlebot4.spawn() with:                            # spawn the robot
                keep(it.spawn_pose.position.x == 0.0m)
                keep(it.spawn_pose.position.y == 0.0m)
                keep(it.spawn_pose.position.z == 0.1m)
                keep(it.spawn_pose.orientation.yaw == 0.0rad)
                keep(it.spawn_pose.orientation.roll == 1.0rad)
"""
        parsed_tree, errors = self.parser.parse_input_stream(InputStream(scenario_content))
        self.assertEqual(errors, 0)
        model = self.parser.create_internal_model(parsed_tree, "test.osc", True)
        self.assertIsNotNone(model)

        pose_struct = model._ModelElement__children[7].get_resolved_value()
        self.assertEqual({'position': {'x': 0., 'y': 0., 'z': 0.}, 'orientation': {
                         'roll': 0., 'pitch': 0., 'yaw': 0.}}, pose_struct)

        behavior = model._ModelElement__children[10]._ModelElement__children[1]._ModelElement__children[0]._ModelElement__children[0].get_resolved_value(
        )
        self.assertEqual({'spawn_pose': {'position': {'x': 0., 'y': 0., 'z': 0.1},
                         'orientation': {'roll': 1., 'pitch': 0., 'yaw': 0.}}}, behavior)
