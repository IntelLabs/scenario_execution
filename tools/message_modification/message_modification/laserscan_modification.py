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

from sensor_msgs.msg import LaserScan
import numpy as np


class LaserscanModification(Node):

    def __init__(self):
        super().__init__('laserscan_modification')

        self.declare_parameter('random_drop_percentage', 0.0)
        self.random_drop_percentage = self.get_parameter('random_drop_percentage').value
        self.declare_parameter('gaussian_noise_std_deviation', 0.0)
        self.gaussian_noise_std_deviation = self.get_parameter('gaussian_noise_std_deviation').value

        self.add_on_set_parameters_callback(self.parameter_callback)

        self.out_publisher = self.create_publisher(LaserScan, "out", qos_profile=QoSPresetProfiles.SENSOR_DATA.value)
        self.in_subscriber = self.create_subscription(LaserScan, 'in', self.callback, qos_profile=QoSPresetProfiles.SENSOR_DATA.value)

    def parameter_callback(self, params):
        result = True
        for param in params:
            if param.name == 'random_drop_percentage' and param.type_ == Parameter.Type.DOUBLE:
                if param.value <= 1.0 and param.value >= 0.0:
                    self.random_drop_percentage = param.value
                else:
                    result = False
            elif param.name == 'gaussian_noise_std_deviation' and param.type_ == Parameter.Type.DOUBLE:
                self.gaussian_noise_std_deviation = param.value
            else:
                result = False

        self.get_logger().info(f"Parameter update {params}: success={result}")
        return SetParametersResult(successful=result)

    def callback(self, msg):
        if self.random_drop_percentage != 0.0:
            ranges = np.asarray(msg.ranges)
            indices = np.random.choice(np.arange(len(msg.ranges)), replace=False,
                                       size=int(len(msg.ranges) * self.random_drop_percentage))
            ranges[indices] = np.inf
            msg.ranges = ranges.tolist()
        if self.gaussian_noise_std_deviation != 0.0:
            gaussian_noise = np.random.normal(0, self.gaussian_noise_std_deviation, len(msg.ranges))
            ranges_with_noise = msg.ranges + gaussian_noise
            msg.ranges = ranges_with_noise.tolist()
        self.out_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    laserscan_modification = LaserscanModification()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(laserscan_modification)

    try:
        executor.spin()
    except KeyboardInterrupt:
        laserscan_modification.get_logger().info("User requested shut down.")
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
