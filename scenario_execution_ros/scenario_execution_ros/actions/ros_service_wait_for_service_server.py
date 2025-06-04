# Copyright (C) 2025 Frederik Pasch
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
from rclpy.node import Node, NodeNameNonExistentError
from scenario_execution.actions.base_action import BaseAction, ActionError


class RosServiceWaitForServiceServer(BaseAction):
    """
    Class to check if a ROS service is provided by a node
    """

    def __init__(self, service: str, node_name: str):
        super().__init__()
        self.service = service
        self.node_name = node_name
        self.node = None

    def setup(self, **kwargs):
        try:
            self.node: Node = kwargs['node']
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(
                self.name, self.__class__.__name__)
            raise ActionError(error_message, action=self) from e

    def update(self) -> py_trees.common.Status:
        for node_name in self.node.get_node_names():
            if not node_name.startswith('/'):
                node_name = '/' + node_name
            if node_name != self.node_name:
                continue
            self.feedback_message = f"detected node {self.node_name}"
            node_base_name = node_name.rsplit('/', 1)[-1]
            namespace = node_name.rsplit('/', 1)[0]
            if namespace == '':
                namespace = '/'
            try:
                services = self.node.get_service_names_and_types_by_node(node_base_name, namespace)
                for service_tuple in services:
                    if service_tuple[0] == self.service:

                        self.feedback_message = f"service {self.service} found in {self.node_name}"  # pylint: disable= attribute-defined-outside-init
                        return py_trees.common.Status.SUCCESS
            except NodeNameNonExistentError:
                pass

        return py_trees.common.Status.RUNNING
