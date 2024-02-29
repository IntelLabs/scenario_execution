#!/usr/bin/env python3
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


import os
import rclpy
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.node import Node

from scenario_execution_interfaces.msg import ScenarioList, Scenario


class ScenarioListPublisher(Node):

    def __init__(self):
        super().__init__('scenario_list_publisher')

        transient_local_qos = QoSProfile(
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1)
        self.publisher = self.create_publisher(
            ScenarioList, '/scenario_execution_control/available_scenarios', transient_local_qos)

        self.declare_parameter('directory', '.')
        scenario_dir = self.get_parameter('directory').value

        scenarios = ScenarioList()
        for f in os.listdir(scenario_dir):
            path = os.path.join(scenario_dir, f)
            if os.path.isfile(path) and path.endswith('.osc'):
                name = os.path.splitext(os.path.basename(path))[0]
                scenarios.scenarios.append(Scenario(name=name, scenario_file=path))
        self.publisher.publish(scenarios)


def main(args=None):
    rclpy.init(args=args)

    node = ScenarioListPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    # rclpy.shutdown()


if __name__ == '__main__':
    main()
