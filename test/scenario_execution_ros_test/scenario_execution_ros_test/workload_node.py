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
from rclpy.node import Node
from rclpy.parameter import Parameter
import sys


class TestNode(Node):

    def __init__(self):
        super().__init__('test_node')
        self.declare_parameter('test_path', rclpy.Parameter.Type.STRING)
        self.declare_parameter('test_param', rclpy.Parameter.Type.STRING)
        self.declare_parameter('timeout', rclpy.Parameter.Type.INTEGER)
        self.test_path = self.get_parameter_or('test_path', '.')
        if isinstance(self.test_path, Parameter):
            self.test_path = self.test_path.value
        self.test_param = self.get_parameter_or('test_param', '')
        if isinstance(self.test_param, Parameter):
            self.test_param = self.test_param.value
        self.timeout = self.get_parameter_or('timeout', 5)
        if isinstance(self.timeout, Parameter):
            self.timeout = self.timeout.value
        self.get_logger().info(f"Writing to '{self.test_path}'... (timeout: {self.timeout})")
        if not os.path.exists(self.test_path):
            os.makedirs(self.test_path)
        with open(os.path.join(self.test_path, 'test_started'), 'w') as f:
            f.write(self.test_param)
        self.timer = self.create_timer(self.timeout, self.timer_callback)

    def timer_callback(self):
        self.get_logger().info('Timeout')
        raise SystemExit()


def main(args=None):
    rclpy.init(args=args)

    result = True
    node = TestNode()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    except BaseException:  # pylint: disable=broad-except
        result = False

    if result:
        print("Success!")
        with open(os.path.join(node.test_path, 'test_success'), 'w') as f:
            f.write(node.test_param)
    else:
        print("Aborted!", file=sys.stderr)
        with open(os.path.join(node.test_path, 'test_aborted'), 'w') as f:
            f.write(node.test_param)


if __name__ == '__main__':
    main()
