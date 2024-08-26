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

import py_trees
from rclpy.node import Node
from scenario_execution.actions.base_action import BaseAction
from ros2node.api import get_node_names


class RosWaitForNodes(BaseAction):
    """
    Wait for nodes to get available
    """

    def __init__(self, nodes: list):
        super().__init__()
        if not isinstance(nodes, list):
            raise TypeError(f'Nodes needs to be list of string, got {type(nodes)}.')
        else:
            self.nodes = nodes
        self.node = None

    def setup(self, **kwargs):
        try:
            self.node: Node = kwargs['node']
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(
                self.name, self.__class__.__name__)
            raise KeyError(error_message) from e

    def update(self) -> py_trees.common.Status:
        available_nodes = get_node_names(node=self.node, include_hidden_nodes=False)
        available_nodes = [seq[2] for seq in available_nodes]
        result = all(elem in available_nodes for elem in self.nodes)
        if result:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.RUNNING
