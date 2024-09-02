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
from scenario_execution.actions.base_action import BaseAction, ActionError


class RosTopicWaitForTopics(BaseAction):
    """
    Class to check if ROS topic are available
    """

    def __init__(self, topics: list):
        super().__init__()
        if not isinstance(topics, list):
            raise TypeError(f'Topics needs to be list of topics, got {type(topics)}.')
        else:
            self.topics = topics
        self.node = None

    def setup(self, **kwargs):
        """
        Setup the publisher
        """
        try:
            self.node: Node = kwargs['node']
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(
                self.name, self.__class__.__name__)
            raise ActionError(error_message, action=self) from e

    def update(self) -> py_trees.common.Status:
        """
        Publish the msg to topic

        return:
            py_trees.common.Status if published
        """
        available_topics = self.node.get_topic_names_and_types()
        available_topics = [seq[0] for seq in available_topics]
        result = all(elem in available_topics for elem in self.topics)
        if result:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.RUNNING
