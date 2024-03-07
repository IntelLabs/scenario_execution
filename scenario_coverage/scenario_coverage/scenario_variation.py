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

import os
import sys
import argparse
from copy import deepcopy

import yaml

from scenario_execution_base.model.osc2_parser import OpenScenario2Parser
from scenario_execution_base.model.model_resolver import resolve_internal_model
from scenario_execution_base.model.types import RelationExpression, ListExpression, print_tree, serialize
from scenario_execution_base.utils.logging import Logger


class ScenarioVariation(object):

    def __init__(self, output_dir, scenario, log_model, debug) -> None:
        self.logger = Logger('scenario_variation')
        self.output_dir = output_dir
        self.scenario = scenario
        self.log_model = log_model
        self.debug = debug

    def get_variations(self, variant_element: RelationExpression):
        base_element = deepcopy(variant_element)
        base_element.operator = '=='
        base_element.delete_child(base_element.get_child_with_expected_type(1, ListExpression))
        variations = []
        list_expression = variant_element.get_child_with_expected_type(1, ListExpression)
        for child in list_expression.get_children():
            variation = deepcopy(base_element)
            variation.set_children(child)
            variations.append(variation)
        return variations

    def run(self) -> bool:
        model = self.load_model()
        if model is None:
            return False

        models = self.generate_concrete_models(model)
        if not models:
            return False

        return self.save_resulting_scenarios(models)

    def load_model(self):
        parser = OpenScenario2Parser(self.logger)
        parsed_tree, errors = parser.parse_file(self.scenario, self.log_model)
        if errors:
            return None

        return parser.load_internal_model(parsed_tree, self.scenario, self.log_model, self.debug)

    def get_next_variation_element(self, elem):
        if isinstance(elem, RelationExpression):
            if elem.operator == 'in':
                return elem
            else:
                return None
        else:
            for child in elem.get_children():
                elem = self.get_next_variation_element(child)
                if elem:
                    return elem
        return None

    def generate_concrete_models(self, model):
        models = [model]
        while True:
            # The following loop always looks at the first element in models.
            # If it contains a variation_element the element is removed and the
            # resulting models are appended at the back.
            variation_element = self.get_next_variation_element(models[0])
            if variation_element is None:
                print("No further variation")
                return models
            print(f"Creating models for variation model {variation_element}")
            # remove model with variation from list
            model = models[0]
            models.remove(model)

            # remove original variation element
            parent = variation_element.get_parent()
            parent.delete_child(variation_element)

            # set resolved variations in copies of original model
            variations = self.get_variations(variation_element)
            for variation in variations:
                parent.set_children(variation)
                variation_model = deepcopy(model)
                models.append(variation_model)
                parent.delete_child(variation)

    def save_resulting_scenarios(self, models):
        idx = 0
        file_path = os.path.join(self.output_dir, os.path.splitext(os.path.basename(self.scenario))[0])
        for model in models:
            print("-----------------")
            test_resolve = deepcopy(model)
            serialize_data = serialize(model)['CompilationUnit']['_children']
            success = resolve_internal_model(test_resolve, self.logger, False)
            if not success:
                print(f"Error: model is not resolvable.")
                return False
            print_tree(model, self.logger)
            filename = file_path + str(idx) + '.sce'
            print(f"Storing model in {filename}")
            with open(filename, 'w') as output:
                yaml.safe_dump(serialize_data, output, sort_keys=False)
            idx += 1
        return True


def main():
    """
    main function
    """
    parser = argparse.ArgumentParser()
    parser.add_argument('-d', '--debug', action='store_true', help='debugging output')
    parser.add_argument('-l', '--log-model', action='store_true', help='Produce tree output of parsed model content')
    parser.add_argument('-o', '--output-dir', type=str, help='Output directory for concrete scenarios', default='out')
    parser.add_argument('scenario', type=str, help='abstract scenario file')
    args = parser.parse_args(sys.argv[1:])

    if not os.path.isdir(args.output_dir):
        os.mkdir(args.output_dir)

    scenario_variation = ScenarioVariation(args.output_dir, args.scenario, args.log_model, args.debug)
    if scenario_variation.run():
        sys.exit(0)
    else:
        print("Error!")
        sys.exit(1)


if __name__ == '__main__':
    main()
