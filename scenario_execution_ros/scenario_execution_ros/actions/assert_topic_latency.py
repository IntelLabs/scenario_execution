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
from rclpy.qos import QoSPresetProfiles
from scenario_execution_ros.actions.conversions import get_comparison_operator


class AssertTopicLatency(py_trees.behaviour.Behaviour):

    def __init__(self, name, topic_name: str, latency: float, comparison_operator: bool, fail_on_finish: bool, rolling_average_count: int, wait_for_first_message: bool):
        super().__init__(name)
        self.topic_name = topic_name
        self.latency = latency
        print(latency)
        self.comparision_operator = get_comparison_operator(comparison_operator)
        self.fail_on_finish = fail_on_finish
        self.rolling_average_count = rolling_average_count
        self.wait_for_first_message = wait_for_first_message
        self.node = None
        self.subscription = None
        self.last_receive_time = None
        self.latency_time = None
        self.msg_count = 0
        self.topic_type = None

    def setup(self, **kwargs):
        try:
            self.node: Node = kwargs['node']
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(
                self.name, self.__class__.__name__)
            raise KeyError(error_message) from e

        topic_available = False
        available_topics = self.node.get_topic_names_and_types()
        for name, topic_type in available_topics:
            if name == self.topic_name:
                self.topic_type = topic_type
                topic_available = True
                break

        if topic_available:
            datatype_in_list = self.topic_type[0].split("/")
            self.topic_type = getattr(
                importlib.import_module(".".join(datatype_in_list[0:-1])),
                datatype_in_list[-1]
            )
        else:
            raise TypeError(f'Topic not available!')

        self.subscription = self.node.create_subscription(
            msg_type=self.topic_type,
            topic=self.topic_name,
            callback=self._callback,
            qos_profile=QoSPresetProfiles.SENSOR_DATA.value)

    def update(self) -> py_trees.common.Status:
        result = py_trees.common.Status.FAILURE
        if self.wait_for_first_message:
            self.feedback_message = f'Waiting for first message to publish on topic {self.topic_name}'
            result = py_trees.common.Status.RUNNING
        else:
            if self.msg_count > 1:
                if self.comparision_operator(self.latency_time, self.latency) and self.fail_on_finish:
                    result = py_trees.common.Status.FAILURE
                elif self.comparision_operator(self.latency_time, self.latency):
                    result = py_trees.common.Status.SUCCESS
                    self.feedback_message = f'Frequency of the topic {self.topic_name}'
                else:
                    result = py_trees.common.Status.RUNNING
                    self.feedback_message = f'Frequency of the topic {self.topic_name}'
        return result

    def _callback(self, msg):
        if self.wait_for_first_message:
            self.wait_for_first_message = False
        else:
            self.msg_count += 1
            now = time.time()
            if self.last_receive_time is not None:
                self.latency_time = now - self.last_receive_time
                print(self.latency_time)
            self.last_receive_time = now
