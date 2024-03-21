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

""" Main entry for scenario_execution_ros """
import sys
from datetime import datetime
import rclpy  # pylint: disable=import-error
import py_trees_ros  # pylint: disable=import-error
from scenario_execution_base import ScenarioExecution
from .logging_ros import RosLogger
from .marker_handler import MarkerHandler


class ROSScenarioExecution(ScenarioExecution):
    """
    Class for scenario execution using ROS2 as middleware
    """

    def __init__(self) -> None:
        self.node = rclpy.create_node(node_name="scenario_execution")
        self.marker_handler = MarkerHandler(self.node)

        # parse from commandline
        args_without_ros = rclpy.utilities.remove_ros_args(sys.argv[1:])
        args = ScenarioExecution.parse_args(args_without_ros)
        debug = args.debug
        log_model = args.log_model
        live_tree = args.live_tree
        scenario = args.scenario
        output_dir = args.output_dir

        # override commandline by ros parameters
        self.node.declare_parameter('debug', False)
        self.node.declare_parameter('log_model', False)
        self.node.declare_parameter('live_tree', False)
        self.node.declare_parameter('output_dir', "")
        self.node.declare_parameter('scenario', "")

        if self.node.get_parameter('debug').value:
            debug = self.node.get_parameter('debug').value
        if self.node.get_parameter('log_model').value:
            log_model = self.node.get_parameter('log_model').value
        if self.node.get_parameter('live_tree').value:
            live_tree = self.node.get_parameter('live_tree').value
        if self.node.get_parameter('scenario').value:
            scenario = self.node.get_parameter('scenario').value
        if self.node.get_parameter('output_dir').value:
            output_dir = self.node.get_parameter('output_dir').value
        super().__init__(debug=debug, log_model=log_model, live_tree=live_tree, scenario_file=scenario, output_dir=output_dir)

    def _get_logger(self, debug):
        """
        Get a logger from ROS2 with name "scenario_execution"
        Overriden parent class method
        """
        return RosLogger('scenario_execution', debug)

    def setup_behaviour_tree(self, tree):
        """
        Setup the behaviour tree
        Using py_trees_ros to get a node handle on ROS2 and tick in syn with ROS2

        Args:
            tree [py_trees.behaviour.Behaviour]: root of the behaviour tree

        return:
            py_trees_ros.trees.BehaviourTree
        """
        return py_trees_ros.trees.BehaviourTree(tree)

    def run(self) -> bool:
        if len(self.scenarios) != 1:
            self.logger.error(f"Only one scenario per file is supported.")
            return False
        self.current_scenario = self.scenarios[0]

        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(self.node)

        self.logger.info(f"Executing scenario '{self.current_scenario.name}'")
        self.current_scenario_start = datetime.now()

        result = self.setup(self.current_scenario, node=self.node, marker_handler=self.marker_handler)

        if result:
            self.behaviour_tree.tick_tock(period_ms=1000. * self.tick_tock_period)
        try:
            executor.spin()
        except KeyboardInterrupt:
            print("Execution got canceled. Exiting...")
            self.on_scenario_shutdown(False, "Aborted")

        return self.process_results()

    def on_scenario_shutdown(self, result, failure_message=""):
        super().on_scenario_shutdown(result, failure_message)
        self.node.destroy_node()
        self.node.executor.create_task(self.node.executor.shutdown)


def main():
    """
    main function
    """
    rclpy.init(args=sys.argv)
    ros_scenario_execution = ROSScenarioExecution()
    result = ros_scenario_execution.parse()

    if result:
        result = ros_scenario_execution.run()

    rclpy.try_shutdown()
    if result:
        sys.exit(0)
    else:
        sys.exit(1)


if __name__ == '__main__':
    main()
