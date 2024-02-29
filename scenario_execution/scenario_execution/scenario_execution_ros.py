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

        # parse from commandline
        args_without_ros = rclpy.utilities.remove_ros_args(sys.argv[1:])
        args = ScenarioExecution.parse_args(args_without_ros)
        debug = args.debug
        log_model = args.log_model
        live_tree = args.live_tree
        scenario = args.scenario
        test_output = args.test_output

        # override commandline by ros parameters
        self.node.declare_parameter('debug', False)
        self.node.declare_parameter('log_model', False)
        self.node.declare_parameter('live_tree', False)
        self.node.declare_parameter('test_output', "")
        self.node.declare_parameter('scenario', "")

        if self.node.get_parameter('debug').value:
            debug = self.node.get_parameter('debug').value
        if self.node.get_parameter('log_model').value:
            log_model = self.node.get_parameter('log_model').value
        if self.node.get_parameter('live_tree').value:
            live_tree = self.node.get_parameter('live_tree').value
        if self.node.get_parameter('scenario').value:
            scenario = self.node.get_parameter('scenario').value
        if self.node.get_parameter('test_output').value:
            test_output = self.node.get_parameter('test_output').value
        super().__init__(debug=debug, log_model=log_model, live_tree=live_tree, scenario=scenario, test_output=test_output)

    def _get_logger(self):
        """
        Get a logger from ROS2 with name "scenario_execution"
        Overriden parent class method
        """
        return RosLogger('scenario_execution')

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
        """
        Setup behaviour tree and run ROS node

        return:
            True if all scenarios are executed successfully
        """
        executor = rclpy.executors.MultiThreadedExecutor()
        marker_handler = MarkerHandler(self.node)
        executor.add_node(self.node)

        if not self.scenarios:
            self.logger.info("No scenarios to execute.")

        failure = False
        for tree in self.scenarios:
            self.logger.info(f"Executing scenario '{tree.name}'")
            start = datetime.now()
            if not tree:
                self.logger.error(f'Scenario {tree.name} has no executables.')
                failure = True
                continue

            result = self.setup(tree, node=self.node, marker_handler=marker_handler)
            if not result:
                failure = True

            if result:
                self.behaviour_tree.tick_tock(period_ms=1000. * self.tick_tock_period)

                # Spin ROS node
                while rclpy.ok() and not self.shutdown_requested:
                    try:
                        # rclpy.spin_once(self.behaviour_tree.node)
                        executor.spin_once()
                    except KeyboardInterrupt:
                        self.blackboard.fail = True
                        break

                self.behaviour_tree.interrupt()
                failure = failure or self.blackboard.fail
                result = not self.blackboard.fail
            if result:
                self.logger.info(f"Scenario '{tree.name}' succeeded.")
            else:
                self.logger.error(f"Scenario '{tree.name}' failed.")
                self.logger.error(self.last_snapshot_visitor.last_snapshot)

            self.add_result((tree.name, self.blackboard.fail, "execution failed",
                            self.last_snapshot_visitor.last_snapshot, datetime.now() - start))
            self.cleanup_behaviours(tree)
            self.behaviour_tree.shutdown()

        return not failure


def main():
    """
    main function
    """
    rclpy.init(args=sys.argv)
    ros_scenario_execution = ROSScenarioExecution()
    result = ros_scenario_execution.parse()

    if result:
        result = ros_scenario_execution.run()
    rclpy.shutdown()
    ros_scenario_execution.report_results()
    if result:
        sys.exit(0)
    else:
        sys.exit(1)


if __name__ == '__main__':
    main()
