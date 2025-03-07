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
import rclpy  # pylint: disable=import-error
import py_trees_ros
from py_trees_ros_interfaces.srv import OpenSnapshotStream
from scenario_execution import ScenarioExecution, ShutdownHandler
from .logging_ros import RosLogger
from .marker_handler import MarkerHandler


class ROSScenarioExecution(ScenarioExecution):
    """
    Class for scenario execution using ROS2 as middleware
    """

    def __init__(self) -> None:
        self.node = rclpy.create_node(node_name="scenario_execution_ros")
        self.marker_handler = MarkerHandler(self.node)
        self.shutdown_task = None

        # parse from commandline
        args_without_ros = rclpy.utilities.remove_ros_args(sys.argv[1:])
        args = ScenarioExecution.parse_args(args_without_ros)
        debug = args.debug
        log_model = args.log_model
        live_tree = args.live_tree
        scenario = args.scenario
        output_dir = args.output_dir
        self.dry_run = args.dry_run
        self.render_dot = args.dot
        self.scenario_parameter_file = args.scenario_parameter_file

        # override commandline by ros parameters
        self.node.declare_parameter('debug', False)
        self.node.declare_parameter('log_model', False)
        self.node.declare_parameter('live_tree', False)
        self.node.declare_parameter('output_dir', "")
        self.node.declare_parameter('scenario', "")
        self.node.declare_parameter('dry_run', False)
        self.node.declare_parameter('dot', False)
        self.node.declare_parameter('scenario_parameter_file', "")

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
        if self.node.get_parameter('dry_run').value:
            self.dry_run = self.node.get_parameter('dry_run').value
        if self.node.get_parameter('dot').value:
            self.render_dot = self.node.get_parameter('dot').value
        if self.node.get_parameter('scenario_parameter_file').value:
            self.scenario_parameter_file = self.node.get_parameter('scenario_parameter_file').value
        self.logger = RosLogger('scenario_execution_ros', debug)
        super().__init__(debug=debug,
                         log_model=log_model,
                         live_tree=live_tree,
                         scenario_file=scenario,
                         output_dir=output_dir,
                         dry_run=self.dry_run,
                         render_dot=self.render_dot,
                         scenario_parameter_file=self.scenario_parameter_file,
                         logger=self.logger)

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

    def post_setup(self):
        request = OpenSnapshotStream.Request()
        request.topic_name = "/scenario_execution/snapshots"
        request.parameters.snapshot_period = sys.float_info.max
        request.parameters.blackboard_data = True
        response = OpenSnapshotStream.Response()
        self.behaviour_tree._open_snapshot_stream(request, response)  # pylint: disable=protected-access

    def run(self) -> bool:

        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(self.node)

        try:
            self.setup(self.tree, node=self.node, marker_handler=self.marker_handler)
        except Exception as e:  # pylint: disable=broad-except
            self.on_scenario_shutdown(False, "Setup failed", f"{e}")
            return

        try:
            self.behaviour_tree.tick_tock(period_ms=1000. * self.tick_period)
            while rclpy.ok():
                try:
                    executor.spin_once(timeout_sec=self.tick_period)
                except KeyboardInterrupt:
                    self.on_scenario_shutdown(False, "Aborted")

                if self.shutdown_task is not None and self.shutdown_task.done():
                    shutdown_handler = ShutdownHandler.get_instance()
                    if shutdown_handler.is_done():
                        self.logger.info("Shutting down finished.")
                        break
        except Exception as e:  # pylint: disable=broad-except
            self.on_scenario_shutdown(False, "Run failed", f"{e}")
        finally:
            rclpy.shutdown()

    def shutdown(self):
        self.logger.info("Shutting down...")
        self.behaviour_tree.shutdown()

    def on_scenario_shutdown(self, result, failure_message="", failure_output=""):
        if self.shutdown_requested:
            return
        super().on_scenario_shutdown(result, failure_message, failure_output)
        self.shutdown_task = self.node.executor.create_task(self.shutdown)


def main():
    """
    main function
    """
    try:
        rclpy.init(args=sys.argv)
        rclpy.uninstall_signal_handlers()
        scenario_execution_ros = ROSScenarioExecution()
    except Exception as e:  # pylint: disable=broad-except
        print(f"Error while initializing: {e}")
        sys.exit(1)

    result = scenario_execution_ros.parse()

    if result and not scenario_execution_ros.dry_run:
        scenario_execution_ros.run()
    result = scenario_execution_ros.process_results()
    rclpy.try_shutdown()
    if result:
        sys.exit(0)
    else:
        sys.exit(1)


if __name__ == '__main__':
    main()
