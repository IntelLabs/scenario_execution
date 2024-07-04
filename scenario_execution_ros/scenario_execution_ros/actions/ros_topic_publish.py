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

import importlib
from ast import literal_eval

from rosidl_runtime_py.set_message import set_message_fields
from rclpy.node import Node

import py_trees
from py_trees.common import Status

from scenario_execution_ros.actions.conversions import get_qos_preset_profile, get_ros_message_type
from scenario_execution.actions.base_action import BaseAction


class RosTopicPublish(BaseAction):
    """
    class for publish a message on a ROS topic
    """

    def __init__(self, topic_type: str, topic_name: str, value: str, qos_profile: tuple):
        super().__init__()
        self.qos_profile = qos_profile
        self.topic_type = topic_type
        self.topic_name = topic_name
        self.publisher = None
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
            raise KeyError(error_message) from e

        topic_type = get_ros_message_type(self.topic_type)
        self.msg_to_pub = topic_type()
        self.publisher = self.node.create_publisher(
            msg_type=topic_type,
            topic=self.topic_name,
            qos_profile=get_qos_preset_profile(self.qos_profile)
        )

    def execute(self, topic_type: str, topic_name: str, value: str, qos_profile: tuple):
        if self.topic_name != topic_name or self.topic_type != topic_type or self.qos_profile != qos_profile:
            raise ValueError("Updating topic parameters not supported.")

        if isinstance(value, str):
            parsed_value = literal_eval("".join(value.split('\\')))
            if not isinstance(parsed_value, dict):
                raise TypeError(f'Parsed value needs type "dict", got {type(parsed_value)}.')
            set_message_fields(self.msg_to_pub, parsed_value)
        elif isinstance(value, dict):
            set_message_fields(self.msg_to_pub, value)
        else:
            self.msg_to_pub = value

    def update(self) -> py_trees.common.Status:
        """
        Publish the msg to topic

        return:
            py_trees.common.Status if published
        """
        self.publisher.publish(self.msg_to_pub)
        self.feedback_message = f"published {self.msg_to_pub}"  # pylint: disable= attribute-defined-outside-init
        return Status.SUCCESS
