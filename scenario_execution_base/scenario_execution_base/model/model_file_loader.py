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


import yaml
from scenario_execution_base.model.types import print_tree, deserialize
from scenario_execution_base.model.model_to_py_tree import create_py_tree
from scenario_execution_base.model.model_resolver import resolve_internal_model


class ModelFileLoader(object):

    def __init__(self, logger) -> None:
        self.logger = logger

    def process_file(self, file_name, log_tree: bool = False, debug: bool = False):
        model = self.load_file(file_name, log_tree)
        resolve_internal_model(model, self.logger, log_tree)
        scenarios = create_py_tree(model, self.logger, log_tree)
        return scenarios

    def load_file(self, file_name, log_tree):
        try:
            with open(file_name, 'rb') as input_file:
                serialize_data = yaml.safe_load(input_file)  # nosec B301 TODO
                model = deserialize(serialize_data)
        except Exception as e:  # pylint: disable=broad-except
            self.logger.error(f"Error while loading model from {file_name}: {e}")
            return None

        if log_tree:
            self.logger.info("----Internal model-----")
            print_tree(model, self.logger)
        return model
