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
Behavior checking if the value matches the key in py_trees blackboard
"""
import py_trees

from py_trees.common import Access, Status


class TopicEquals(py_trees.behaviour.Behaviour):
    """
    Class to listen to a topic in Blackboard and check if it equals the defined message

    Args:
        key [str]: topic to listen to
        msg [str]: target message to match
        namespace [str]: namespace of the key
    """

    def __init__(self, key: str, msg: str, namespace: str = None):
        super().__init__(self.__class__.__name__)

        self.namespace = namespace
        self.key = key
        self.msg = msg

        self.client = self.attach_blackboard_client(namespace=self.namespace)
        self.client.register_key(self.key, access=Access.READ)

    def update(self):
        """
        Check the message on the topic equals the target message
        """
        msg_on_blackboard = self.client.get(self.key)
        if msg_on_blackboard == self.msg:
            return Status.SUCCESS
        return Status.RUNNING
