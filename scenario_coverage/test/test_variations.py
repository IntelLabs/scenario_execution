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

from scenario_coverage.scenario_variation import ScenarioVariation
from scenario_execution.model.model_file_loader import ModelFileLoader
from scenario_execution.model.model_resolver import resolve_internal_model
from scenario_execution.utils.logging import Logger
import tempfile
import copy
import os
import py_trees


class TestOSC2Parser(unittest.TestCase):
    # pylint: disable=missing-function-docstring, protected-access, no-member, unused-variable
    def setUp(self) -> None:
        self.tmpdir = tempfile.TemporaryDirectory()
        self.tree = py_trees.composites.Sequence(name="", memory=True)

    def run_coverage(self, scenario_content):
        fp = tempfile.NamedTemporaryFile(suffix='.osc', mode='w', delete=False)
        fp.write(scenario_content)
        fp.close()
        coverage = ScenarioVariation(self.tmpdir.name, fp.name, False, False)
        model = coverage.load_model()
        del fp
        return model, coverage

    def test_struct_variation(self):
        scenario_content = """
        
struct base_struct:
    base1: string
    base2: string = "predefined"

struct test_struct: 
    member1: string
    member2: base_struct

action foo:
    param1: test_struct

scenario test:
    do serial:
        foo() with:
            keep(it.param1 in [test_struct(member1: 'test1', member2: base_struct(base1: 'foo1')), test_struct(member1: 'test2', member2: base_struct(base1: 'foo2'))])
"""
        model, coverage = self.run_coverage(scenario_content)
        self.assertIsNotNone(model)
        models = coverage.generate_concrete_models(model)
        self.assertIsNotNone(models)
        self.assertEqual(2, len(models))

        # Model 1
        model = copy.deepcopy(models[0][0])
        resolve_internal_model(model, self.tree, Logger('test', False), False)
        elem = model._ModelElement__children[3]._ModelElement__children[0]._ModelElement__children[0]._ModelElement__children[0]
        value = elem.get_resolved_value()
        self.assertEqual(value, {'param1': {'member1': 'test1', 'member2': {'base1': 'foo1', 'base2': 'predefined'}}})

        # Model 2
        model = copy.deepcopy(models[1][0])
        resolve_internal_model(model, self.tree, Logger('test', False), False)
        elem = model._ModelElement__children[3]._ModelElement__children[0]._ModelElement__children[0]._ModelElement__children[0]
        value = elem.get_resolved_value()
        self.assertEqual(value, {'param1': {'member1': 'test2', 'member2': {'base1': 'foo2', 'base2': 'predefined'}}})

        result = coverage.save_resulting_scenarios(models)
        self.assertTrue(result)

        dir_content = os.listdir(self.tmpdir.name)
        scenarios = []
        for entry in dir_content:
            if entry.endswith(".sce"):
                scenarios.append(os.path.join(self.tmpdir.name, entry))
        self.assertEqual(2, len(scenarios))

        # read from file
        scenario0 = None
        for scenario in scenarios:
            if scenario.endswith("0.sce"):
                scenario0 = scenario
        parser = ModelFileLoader(Logger('test', False))
        model = parser.load_file(scenario0, True)
        self.assertIsNotNone(model)
        resolve_internal_model(model, self.tree, parser.logger, False)
        elem = model._ModelElement__children[3]._ModelElement__children[0]._ModelElement__children[0]._ModelElement__children[0]
        value = elem.get_resolved_value()
        self.assertEqual(value, {'param1': {'member1': 'test1', 'member2': {'base1': 'foo1', 'base2': 'predefined'}}})

    def test_multi_variation(self):
        scenario_content = """
action foo:
    param1: string
    param2: string

scenario test:
    do serial:
        foo() with:
            keep(it.param1 in ['A1', 'A2'])
            keep(it.param2 in ['B1', 'B2', 'B3'])
"""
        model, coverage = self.run_coverage(scenario_content)
        self.assertIsNotNone(model)
        models = coverage.generate_concrete_models(model)
        self.assertIsNotNone(models)
        self.assertEqual(6, len(models))

        # Model 1
        model = copy.deepcopy(models[0][0])
        resolve_internal_model(model, self.tree, Logger('test', False), False)
        elem = model._ModelElement__children[1]._ModelElement__children[0]._ModelElement__children[0]._ModelElement__children[0]
        value = elem.get_resolved_value()
        self.assertEqual(value, {'param1': 'A1', 'param2': 'B1'})
        self.assertEqual(len(models[0][1]), 2)

        # Model 2
        model = copy.deepcopy(models[1][0])
        resolve_internal_model(model, self.tree, Logger('test', False), False)
        elem = model._ModelElement__children[1]._ModelElement__children[0]._ModelElement__children[0]._ModelElement__children[0]
        value = elem.get_resolved_value()
        self.assertEqual(value, {'param1': 'A1', 'param2': 'B2'})
        self.assertEqual(len(models[0][1]), 2)

        # Model 3
        model = copy.deepcopy(models[2][0])
        resolve_internal_model(model, self.tree, Logger('test', False), False)
        elem = model._ModelElement__children[1]._ModelElement__children[0]._ModelElement__children[0]._ModelElement__children[0]
        value = elem.get_resolved_value()
        self.assertEqual(value, {'param1': 'A1', 'param2': 'B3'})
        self.assertEqual(len(models[0][1]), 2)

        # Model 4
        model = copy.deepcopy(models[3][0])
        resolve_internal_model(model, self.tree, Logger('test', False), False)
        elem = model._ModelElement__children[1]._ModelElement__children[0]._ModelElement__children[0]._ModelElement__children[0]
        value = elem.get_resolved_value()
        self.assertEqual(value, {'param1': 'A2', 'param2': 'B1'})
        self.assertEqual(len(models[0][1]), 2)

        # Model 5
        model = copy.deepcopy(models[4][0])
        resolve_internal_model(model, self.tree, Logger('test', False), False)
        elem = model._ModelElement__children[1]._ModelElement__children[0]._ModelElement__children[0]._ModelElement__children[0]
        value = elem.get_resolved_value()
        self.assertEqual(value, {'param1': 'A2', 'param2': 'B2'})
        self.assertEqual(len(models[0][1]), 2)

        # Model 6
        model = copy.deepcopy(models[5][0])
        resolve_internal_model(model, self.tree, Logger('test', False), False)
        elem = model._ModelElement__children[1]._ModelElement__children[0]._ModelElement__children[0]._ModelElement__children[0]
        value = elem.get_resolved_value()
        self.assertEqual(value, {'param1': 'A2', 'param2': 'B3'})
        self.assertEqual(len(models[0][1]), 2)
