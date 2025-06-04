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
from rclpy.node import Node
from scenario_execution.actions.base_action import BaseAction, ActionError


class RosTopicWaitForTopicSubscription(BaseAction):
    """
    Class to check if a specific node is subscribed to a ROS topic
    """

    def __init__(self, topic: str, node_name: str):
        super().__init__()
        self.topic = topic
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
        subscribers_info = self.node.get_subscriptions_info_by_topic(self.topic)
        subscriber_node_names = [info.node_name for info in subscribers_info]

        if self.node_name in subscriber_node_names:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.RUNNING
