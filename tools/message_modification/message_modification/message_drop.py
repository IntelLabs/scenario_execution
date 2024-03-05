#!/usr/bin/env python
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

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles

from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult

import importlib
import random


class MessageDrop(Node):

    def __init__(self):
        super().__init__('message_drop')

        self.declare_parameter('message_type', rclpy.Parameter.Type.STRING)
        datatype_in_list = self.get_parameter('message_type').value.split(".")
        self.topic_type = getattr(importlib.import_module(".".join(datatype_in_list[0:-1])), datatype_in_list[-1])

        self.declare_parameter('drop_rate', 0.0)
        self.drop_rate = self.get_parameter('drop_rate').value

        self.out_publisher = self.create_publisher(self.topic_type, "out", qos_profile=QoSPresetProfiles.SENSOR_DATA.value)
        self.in_subscriber = self.create_subscription(self.topic_type, 'in', self.callback, qos_profile=QoSPresetProfiles.SENSOR_DATA.value)

    def parameter_callback(self, params):
        result = True
        for param in params:
            if param.name == 'drop_rate' and param.type_ == Parameter.Type.DOUBLE:
                if param.value <= 1.0:
                    self.drop_rate = param.value
                else:
                    result = False
            else:
                result = False
        self.get_logger().info(f"Parameter update {params}. result={result}")
        return SetParametersResult(successful=result)

    def callback(self, msg):
        drop = random.random() <= self.drop_rate  # nosec B311
        if not drop:
            self.out_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MessageDrop()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("User requested shut down.")
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
