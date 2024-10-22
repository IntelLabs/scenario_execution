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
import random as rd


def seed(seed_value: int = 0):
    rd.seed(seed_value)  # nosec B311


def get_float(min_val: dict, max_val: float):
    return rd.uniform(min_val, max_val)  # nosec B311


def get_int(min_val: int, max_val: int):
    return rd.randint(min_val, max_val)  # nosec B311


def get_random_list_element(elements_list: list):
    if not elements_list:
        return None  # Return None if the list is empty
    return rd.choice(elements_list)  # nosec B311
