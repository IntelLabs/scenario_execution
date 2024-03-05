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
Test for Intermediate Scenario model Serialization and Deserialization
"""
import unittest

from scenario_coverage.scenario_variation import ScenarioVariation
from scenario_execution_base.model.model_file_loader import ModelFileLoader
from scenario_execution_base.utils.logging import BaseLogger, Logger
from scenario_execution_base.model.types import print_tree
import tempfile
import os


class DebugLogger(BaseLogger):

    def __init__(self, name):
        super().__init__(name)
        self.logs = []

    def info(self, msg: str) -> None:
        self.logs.append(msg)

    def debug(self, msg: str) -> None:
        self.logs.append(msg)

    def warning(self, msg: str) -> None:
        self.logs.append(msg)

    def error(self, msg: str) -> None:
        self.logs.append(msg)

    def count_lines(self):
        return len(self.logs)


class TestOSC2Parser(unittest.TestCase):
    # pylint: disable=missing-function-docstring, protected-access, no-member, unused-variable
    def setUp(self) -> None:
        self.tmpdir = tempfile.TemporaryDirectory()

    def run_coverage(self, scenario_content):
        fp = tempfile.NamedTemporaryFile(suffix='.osc', mode='w', delete=False)
        fp.write(scenario_content)
        fp.close()
        coverage = ScenarioVariation(self.tmpdir.name, fp.name, False, False)
        model = coverage.load_model()
        del fp
        return model, coverage

    def test_serialize(self):
        scenario_content = """
import osc.standard

action log:
    msg: string

scenario test:
    do serial:
        log() with:
            keep(it.msg in ["foo", "bar"])
        emit end
"""
        # serialize
        model, coverage = self.run_coverage(scenario_content)
        self.assertIsNotNone(model)
        models = coverage.generate_concrete_models(model)
        self.assertIsNotNone(models)
        self.assertEqual(2, len(models))
        result = coverage.save_resulting_scenarios(models)
        self.assertTrue(result)

        # deserialize
        dir_content = os.listdir(self.tmpdir.name)
        scenarios = []
        for entry in dir_content:
            if entry.endswith(".sce"):
                scenarios.append(os.path.join(self.tmpdir.name, entry))
        self.assertEqual(2, len(scenarios))
        print(scenarios)
        scenario0 = None
        scenario1 = None
        for scenario in scenarios:
            if scenario.endswith("0.sce"):
                scenario0 = scenario
            if scenario.endswith("1.sce"):
                scenario1 = scenario
        parser = ModelFileLoader(Logger('test'))
        models_deserialized = []
        model0 = parser.load_file(scenario0, False)
        models_deserialized.append(model0)
        model1 = parser.load_file(scenario1, False)
        models_deserialized.append(model1)
        self.assertIsNotNone(models_deserialized)
        self.assertEqual(2, len(models_deserialized))

        # validate models
        deserialize_model_0 = DebugLogger('deserialize_model_0')
        base_model_0 = DebugLogger('base_model_0')
        print_tree(models_deserialized[0], deserialize_model_0)
        print_tree(models[0], base_model_0)
        self.assertListEqual(deserialize_model_0.logs, base_model_0.logs)

        deserialize_model_1 = DebugLogger('deserialize_model_1')
        base_model_1 = DebugLogger('base_model_1')
        print_tree(models_deserialized[1], deserialize_model_1)
        print_tree(models[1], base_model_1)
        self.assertListEqual(deserialize_model_1.logs, base_model_1.logs)

        # test model lines
        self.assertGreater(deserialize_model_0.count_lines(), 0)
