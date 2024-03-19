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

"""
Execute scenarios via ros service
"""

import os
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy

from scenario_execution_control.application_runner import ApplicationStatus  # pylint: disable=relative-import
from scenario_execution_control.scenario_execution_runner import ScenarioExecutionRunner  # pylint: disable=relative-import

from std_srvs.srv import Empty
from scenario_execution_interfaces.srv import ExecuteScenario
from scenario_execution_interfaces.msg import ScenarioExecutionStatus


class ScenarioExecutionControl(Node):
    """
    Execute scenarios via ros service
    """

    def __init__(self):
        """
        Constructor
        """
        super(ScenarioExecutionControl, self).__init__('scenario_execution_control')
        self.shutdown_requested = False
        self._status_publisher = self.create_publisher(
            ScenarioExecutionStatus,
            "/scenario_execution_control/status",
            qos_profile=QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.scenario_execution_status_updated(ApplicationStatus.STOPPED)
        self._scenario_execution = ScenarioExecutionRunner(
            self.scenario_execution_status_updated,
            self.scenario_execution_log)
        self._execute_scenario_service = self.create_service(
            ExecuteScenario,
            '/scenario_execution_control/execute_scenario',
            self.execute_scenario)
        self._stop_scenario_service = self.create_service(
            Empty,
            '/scenario_execution_control/stop_scenario',
            self.stop_scenario)
        self.declare_parameter('output_directory', '.')
        self.output_directory = ""
        if self.get_parameter('output_directory').value:
            self.output_directory = "-o " + self.get_parameter('output_directory').value

    def scenario_execution_log(self, log):  # pylint: disable=no-self-use
        """
        Callback for application logs
        """
        self.get_logger().warn(f"[SC]{log}")

    def scenario_execution_status_updated(self, status):
        """
        Executed from application runner whenever the status changed
        """
        self.get_logger().info(f"Status updated to {status}")
        val = ScenarioExecutionStatus.STOPPED
        if status == ApplicationStatus.STOPPED:
            val = ScenarioExecutionStatus.STOPPED
        elif status == ApplicationStatus.STARTING:
            val = ScenarioExecutionStatus.STARTING
        elif status == ApplicationStatus.RUNNING:
            val = ScenarioExecutionStatus.RUNNING
        elif status == ApplicationStatus.SHUTTINGDOWN:
            val = ScenarioExecutionStatus.SHUTTINGDOWN
        else:
            if self.shutdown_requested:
                val = ScenarioExecutionStatus.STOPPED
            else:
                val = ScenarioExecutionStatus.ERROR
        status = ScenarioExecutionStatus()
        status.status = val
        self._status_publisher.publish(status)

    def execute_scenario(self, req, response=None):
        """
        Execute a scenario
        """
        self.get_logger().info(f"Scenario Execution requested ({req.scenario.scenario_file})...")

        response = ExecuteScenario.Response()
        response.result = True
        if not os.path.isfile(req.scenario.scenario_file):
            self.get_logger().warn(
                f"Requested scenario file not existing {req.scenario.scenario_file}")
            response.result = False
        else:
            self.executor.create_task(self.run, req.scenario)
        return response

    def stop_scenario(self, _, response=None):
        """
        Stop current scenario
        """
        response = Empty.Response()

        if self._scenario_execution.is_running():
            self.get_logger().info(f"Scenario Stop requested...")
            self.shutdown_requested = True
            self._scenario_execution.shutdown()
            self.get_logger().info("Scenario Execution stopped.")
        else:
            self.get_logger().info("Scenario Stop requested, but not Scenario running.")
        return response

    def run(self, task):
        current_req = task
        if self._scenario_execution.is_running():
            self.get_logger().info("Scenario Execution currently running. Shutting down.")
            self._scenario_execution.shutdown()
            self.get_logger().info("Scenario Execution stopped.")
        self.get_logger().info(f"Executing scenario ({current_req.scenario_file})...")

        # execute scenario
        self.shutdown_requested = False
        scenario_executed = self._scenario_execution.execute_scenario(current_req.scenario_file, self.output_directory)
        if not scenario_executed:
            self.get_logger().warn("Unable to execute scenario.")


def main(args=None):
    """

    main function

    :return:
    """
    rclpy.init(args=args)
    scenario_execution_control = ScenarioExecutionControl()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(scenario_execution_control)

    try:
        executor.spin()
    except KeyboardInterrupt:
        scenario_execution_control.get_logger().info("User requested shut down.")
    finally:
        if scenario_execution_control._scenario_execution.is_running():  # pylint: disable=protected-access
            scenario_execution_control.get_logger().info("Scenario Execution still running. Shutting down.")
            scenario_execution_control._scenario_execution.shutdown()  # pylint: disable=protected-access
        del scenario_execution_control

        rclpy.shutdown()


if __name__ == "__main__":
    main()
