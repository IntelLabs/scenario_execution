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


import py_trees  # pylint: disable=import-error
from rclpy.node import Node
import importlib
import time
from scenario_execution_ros.actions.conversions import get_comparison_operator, get_qos_preset_profile


class AssertTfMoving(py_trees.behaviour.Behaviour):

    def __init__(self, name, frame_id: str, parent_frame_id: str, timeout: int, threshold_speed: bool, fail_on_finish: bool, wait_for_first_transform: bool):
        super().__init__(name)
        self.frame_id = frame_id
        self.parent_frame_id = parent_frame_id
        self.timeout = timeout
        self.fail_on_finish = fail_on_finish
        self.threshold_speed = threshold_speed
        self.wait_for_first_transform = wait_for_first_transform
        self.node = None
        self.subscription = None

    def setup(self, **kwargs):
        try:
            self.node: Node = kwargs['node']
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(
                self.name, self.__class__.__name__)
            raise KeyError(error_message) from e

    def update(self) -> py_trees.common.Status:
        result = py_trees.common.Status.FAILURE
        return result
