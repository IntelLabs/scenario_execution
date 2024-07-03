#! /usr/bin/env python3

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
import py_trees
from scenario_execution.model.osc2_parser import OpenScenario2Parser
from scenario_execution.model.model_resolver import resolve_internal_model
from scenario_execution.model.types import RelationExpression, ListExpression, FieldAccessExpression, Expression, print_tree, serialize, to_string
from scenario_execution.utils.logging import Logger


class ScenarioVariation(object):

    def __init__(self, output_dir, scenario, log_model, debug) -> None:
        self.logger = Logger('scenario_variation', debug)
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
            variations.append((variation, child))
        return variations

    def run(self) -> bool:
        model = self.load_model()
        models = self.generate_concrete_models(model)
        return self.save_resulting_scenarios(models)

    def load_model(self):
        parser = OpenScenario2Parser(self.logger)
        parsed_tree = parser.parse_file(self.scenario, self.log_model)
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

    def get_element_fully_qualified_name(self, element):
        def get_name(elem):
            if elem.name:
                return elem.name
            else:
                name = elem.__class__.__name__
                if elem.has_siblings():
                    idx = list(elem.get_parent().get_children()).index(elem)
                    name += f"[{idx}]"
                return name

        fqn = get_name(element)
        tmp = element.get_parent()
        while tmp:
            fqn = get_name(tmp) + "." + fqn
            tmp = tmp.get_parent()
        return fqn

    def generate_concrete_models(self, model):
        models = [(model, [])]  # model and variation description as tuple
        while True:
            # The following loop always looks at the first element in models.
            # If it contains a variation_element the element is removed and the
            # resulting models are appended at the back.
            variation_element = self.get_next_variation_element(models[0][0])
            if variation_element is None:
                self.logger.debug("No further variation")
                return models
            self.logger.info(f"Creating models for variation model {variation_element}")
            # remove model with variation from list
            model = models[0]
            models.remove(model)

            # remove original variation element
            parent = variation_element.get_parent()
            parent.delete_child(variation_element)

            # set resolved variations in copies of original model
            variations = self.get_variations(variation_element)
            for variation in variations:
                parent.set_children(variation[0])
                variation_model = deepcopy(model)

                fqn = self.get_element_fully_qualified_name(variation[0])
                variation_model[1].append((fqn, variation[0]))
                models.append(variation_model)
                parent.delete_child(variation[0])

    def save_resulting_scenarios(self, models):
        idx = 0
        file_path = os.path.join(self.output_dir, os.path.splitext(os.path.basename(self.scenario))[0])
        for model in models:
            self.logger.debug("-----------------")
            serialize_data = serialize(model[0])['CompilationUnit']['_children']
            if self.debug:
                print_tree(model[0], self.logger)
            try:
                tree = py_trees.composites.Sequence()
                resolve_internal_model(model[0], tree, self.logger, False)
            except ValueError as e:
                raise ValueError(f"Resulting model is not resolvable: {e}") from e

            # create description
            variation_descriptions = []
            for descr, entry in model[1]:
                if isinstance(entry, Expression):
                    val = None
                    for child in entry.get_children():
                        if not isinstance(child, FieldAccessExpression):
                            val = child
                    if val is None:
                        raise ValueError("Could not find value.")
                    value_string = to_string(val)
                else:
                    raise ValueError("Can not write variation description")
                variation_descriptions.append(f"{descr}=={value_string}")
            filename = file_path + str(idx) + '.sce'
            self.logger.info(f"Storing model in {filename}")
            with open(filename, 'w') as output:
                for descr in variation_descriptions:
                    output.write(f"#{descr}\n")
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
    try:
        ret = scenario_variation.run()
    except Exception as e:  # pylint: disable=broad-except
        scenario_variation.logger.error(e)
        ret = False

    if ret:
        sys.exit(0)
    else:
        print("Error!")
        sys.exit(1)


if __name__ == '__main__':
    main()
