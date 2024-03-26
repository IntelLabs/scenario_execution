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
from scenario_execution_interfaces.srv import ExecuteScenario
from scenario_execution_interfaces.msg import ScenarioExecutionStatus
import time
import argparse
import sys


class ScenarioExecutionControlTest(Node):
    """
    Test working of scenario execution control (eg. for CI)
    """

    def __init__(self, service_timeout):
        super().__init__('execute_scenario')
        self.execute_scenario_client = self.create_client(ExecuteScenario, '/scenario_execution_control/execute_scenario')
        end_time = time.time() + service_timeout
        while not self.execute_scenario_client.wait_for_service(timeout_sec=1.0) and time.time() < end_time:
            self.get_logger().info('Service not available, waiting again...')
        if time.time() >= end_time:
            rclpy.shutdown()
            raise RuntimeError(f'Service not available after {service_timeout} seconds.')

        self.req = ExecuteScenario.Request()

        # Initialize subscriber for scenario status
        self.subscription = self.create_subscription(
            ScenarioExecutionStatus,
            '/scenario_execution_control/status',
            self.scenario_execution_status_callback,
            10)
        self.scenario_status = None
        self.future = None

    def execute_scenario(self, scenario_file):
        self.req.scenario.name = "test_scenario"
        self.req.scenario.scenario_file = scenario_file
        self.future = self.execute_scenario_client.call_async(self.req)

    def scenario_execution_status_callback(self, msg):
        self.scenario_status = msg.status
        self.get_logger().info(f'Scenario status: {self.scenario_status}')

    @staticmethod
    def parse_args(args):
        parser = argparse.ArgumentParser()
        parser.add_argument('scenario', type=str, help='scenario file to execute')
        args, _ = parser.parse_known_args(args)
        return args


def main(args=None):
    """

    main function

    :return:
    """
    args = ScenarioExecutionControlTest.parse_args(sys.argv[1:])
    rclpy.init()
    service_timeout = 60  # seconds
    scenario_execute_control_test = ScenarioExecutionControlTest(service_timeout)
    if not scenario_execute_control_test.execute_scenario_client.service_is_ready():
        sys.exit(1)
    scenario_execute_control_test.execute_scenario(args.scenario)

    start_time = time.time()
    timeout = 60  # seconds
    while rclpy.ok():
        rclpy.spin_once(scenario_execute_control_test, timeout_sec=0.1)
        if scenario_execute_control_test.future.done():
            try:
                scenario_execute_control_test.future.result()
            except RuntimeError as e:
                scenario_execute_control_test.get_logger().info(f'Service call failed: {e}')
                sys.exit(1)
            else:
                if scenario_execute_control_test.scenario_status == 0:
                    scenario_execute_control_test.get_logger().info(f'Success! Scenario Succeed')
                    sys.exit(0)
                else:
                    if time.time() - start_time > timeout:
                        scenario_execute_control_test.get_logger().info('Status check timed out.')
                        sys.exit(1)
                    else:
                        scenario_execute_control_test.get_logger().info('Waiting for success status...')

    scenario_execute_control_test.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
