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


from collections import deque
import py_trees  # pylint: disable=import-error
from rclpy.node import Node
import importlib
import time
from scenario_execution_ros.actions.conversions import get_comparison_operator, get_qos_preset_profile
from scenario_execution.actions.base_action import BaseAction, ActionError


class AssertTopicLatency(BaseAction):

    def __init__(self, topic_name: str, topic_type: str, wait_for_first_message: bool, type_check: bool):
        super().__init__()
        self.topic_name = topic_name
        self.topic_type = topic_type
        self.type_check = type_check
        self.latency = None
        self.comparison_operator_feedback = None
        self.comparison_operator = None
        self.rolling_average_count = None
        self.rolling_average_count_queue = None
        self.wait_for_first_message = wait_for_first_message
        self.initial_wait_for_first_message = wait_for_first_message
        self.first_message_received = False
        self.node = None
        self.subscription = None
        self.last_receive_time = 0
        self.msg_count = 0
        self.average_latency = 0.
        self.timer = time.time()
        self.is_topic = False
        self.retrigger = None

    def setup(self, **kwargs):
        try:
            self.node: Node = kwargs['node']
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(
                self.name, self.__class__.__name__)
            raise ActionError(error_message, action=self) from e

        success = self.check_topic()
        if not success and self.wait_for_first_message:
            raise ActionError("Invalid topic or type specified.", action=self)
        elif not success and not self.wait_for_first_message:
            raise ActionError("Topic type must be specified. Please provide a valid topic type.", action=self)

    def execute(self, latency: float, comparison_operator: bool, rolling_average_count: int):
        self.latency = latency
        self.comparison_operator_feedback = comparison_operator[0]
        self.comparison_operator = get_comparison_operator(comparison_operator)
        self.rolling_average_count = rolling_average_count
        self.rolling_average_count_queue = deque(maxlen=rolling_average_count)
        self.timer = time.time()
        self.last_receive_time = 0
        self.msg_count = 0
        self.average_latency = 0.
        self.is_topic = False
        self.retrigger = True

    def update(self) -> py_trees.common.Status:
        result = py_trees.common.Status.FAILURE
        if self.retrigger:
            self.wait_for_first_message = self.initial_wait_for_first_message
            self.retrigger = False
        if not self.is_topic:
            self.check_topic()
            self.logger.info(f"Waiting for the topic '{self.topic_name}' to become available")
            self.feedback_message = f"Waiting for the topic '{self.topic_name}' to become available"  # pylint: disable= attribute-defined-outside-init
            result = py_trees.common.Status.RUNNING
        else:
            if self.wait_for_first_message:
                self.logger.info(f"Waiting for first message to publish on topic {self.topic_name}")
                self.feedback_message = f'Waiting for first message to publish on topic {self.topic_name}'  # pylint: disable= attribute-defined-outside-init
                result = py_trees.common.Status.RUNNING
            else:
                if not self.first_message_received:
                    if time.time() - self.timer > self.latency:
                        self.feedback_message = f"Failed to receive message within the expected latency threshold ({self.latency} seconds)"  # pylint: disable= attribute-defined-outside-init
                        result = py_trees.common.Status.FAILURE
                    else:
                        self.feedback_message = f"No message received on the topic '{self.topic_name}'"  # pylint: disable= attribute-defined-outside-init
                        result = py_trees.common.Status.RUNNING
                elif self.msg_count > 1:
                    if self.comparison_operator(self.latency, time.time() - self.last_receive_time):
                        self.feedback_message = f"Failed to receive message within the expected latency threshold ({self.latency} seconds)"  # pylint: disable= attribute-defined-outside-init
                        result = py_trees.common.Status.FAILURE
                    elif self.comparison_operator(self.average_latency, self.latency):
                        result = py_trees.common.Status.RUNNING
                        self.feedback_message = f'Latency within range: expected {self.comparison_operator_feedback} {self.latency} s, actual {self.average_latency} s'  # pylint: disable= attribute-defined-outside-init
                    else:
                        result = py_trees.common.Status.FAILURE
                        self.feedback_message = f'Latency not within range: expected {self.comparison_operator_feedback} {self.latency} s, actual {self.average_latency} s'  # pylint: disable= attribute-defined-outside-init
                else:
                    result = py_trees.common.Status.RUNNING
        return result

    def check_topic(self):
        result = True
        if not self.wait_for_first_message:
            if not self.topic_type:
                result = False  # Topic type must be specified. (wait_for_first_message == False)
            else:
                self.call_subscriber()
                self.is_topic = True  # wait_for_first_message' is set to false, the process will proceed with the specified topic and type
        else:
            available_topics = self.node.get_topic_names_and_types()
            topic_found = False
            for name, topic_types in available_topics:
                if name == self.topic_name:
                    topic_found = True
                    if not topic_types:
                        result = False
                        break
                    current_topic_type = topic_types[0].replace('/', '.')
                    if not self.topic_type:
                        self.topic_type = current_topic_type  # 'topic_type' is not specified, the process will continue with the found one
                        self.call_subscriber()
                        self.is_topic = True
                    elif self.type_check and self.topic_type != current_topic_type:
                        result = False
                    else:
                        self.call_subscriber()
                        self.is_topic = True
                    break
            if not topic_found:
                pass  # Topic not available wait....
        return result

    def call_subscriber(self):
        datatype_in_list = self.topic_type.split(".")
        topic_type = getattr(
            importlib.import_module(".".join(datatype_in_list[:-1])),
            datatype_in_list[-1]
        )

        self.subscription = self.node.create_subscription(
            msg_type=topic_type,
            topic=self.topic_name,
            callback=self._callback,
            qos_profile=get_qos_preset_profile(['sensor_data']))

    def _callback(self, msg):
        self.first_message_received = True
        now = time.time()
        if self.wait_for_first_message:
            self.wait_for_first_message = False
        else:
            self.msg_count += 1
            if self.last_receive_time:
                latency_time = now - self.last_receive_time
                if len(self.rolling_average_count_queue) == self.rolling_average_count:
                    self.rolling_average_count_queue.popleft()
                self.rolling_average_count_queue.append(latency_time)
                self.average_latency = sum(self.rolling_average_count_queue) / len(self.rolling_average_count_queue)
            self.last_receive_time = now
