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
import sys
import time
import argparse
import signal
from datetime import datetime, timedelta
import py_trees
from scenario_execution_base.model.osc2_parser import OpenScenario2Parser
from scenario_execution_base.utils.logging import Logger
from scenario_execution_base.model.model_file_loader import ModelFileLoader
from dataclasses import dataclass


@dataclass
class ScenarioResult:
    name: str
    result: bool
    failure_message: str
    failure_output: str = ""
    processing_time: timedelta = timedelta(0)


class LastSnapshotVisitor(py_trees.visitors.DisplaySnapshotVisitor):

    def __init__(self):
        self.last_snapshot = ""
        super().__init__()

    def finalise(self) -> None:
        if self.root is not None:
            self.last_snapshot = py_trees.display.unicode_tree(
                root=self.root,
                show_only_visited=self.display_only_visited_behaviours,
                show_status=False,
                visited=self.visited,
                previously_visited=self.previously_visited
            )


class ScenarioExecution(object):
    """
    Base class for scenario execution.
    Override method run() and method setup_behaviour_tree() to adapt to other middlewares.
    This class can also be executed standalone
    """

    def __init__(self,
                 debug: bool,
                 log_model: bool,
                 live_tree: bool,
                 scenario_file: str,
                 output_dir: str,
                 setup_timeout=py_trees.common.Duration.INFINITE,
                 tick_tock_period: float = 0.1) -> None:

        def signal_handler(sig, frame):
            self.on_scenario_shutdown(False, "Aborted")

        signal.signal(signal.SIGHUP, signal_handler)
        signal.signal(signal.SIGTERM, signal_handler)

        self.current_scenario_start = None
        self.current_scenario = None
        self.debug = debug
        self.log_model = log_model
        self.live_tree = live_tree
        self.scenario_file = scenario_file
        self.output_dir = output_dir

        if self.output_dir:
            if not os.path.isdir(self.output_dir):
                try:
                    os.mkdir(self.output_dir)
                except OSError as e:
                    raise ValueError(f"Could not create output directory: {e}") from e
            if not os.access(self.output_dir, os.W_OK):
                raise ValueError(f"Output directory '{self.output_dir}' not writable.")

        self.logger = self._get_logger(debug)

        if self.debug:
            py_trees.logging.level = py_trees.logging.Level.DEBUG
        self.setup_timeout = setup_timeout
        self.tick_tock_period = tick_tock_period
        self.scenarios = None
        self.blackboard = None
        self.behaviour_tree = None
        self.last_snapshot_visitor = None
        self.shutdown_requested = False
        self.results = []

    def _get_logger(self, debug):
        """
        Create a logger
        This method could be overriden by child classes and defined according to the middleware.

        return:
            A logger which has three logging levels: info, warning, error
        """
        return Logger('scenario_execution', debug)

    def setup(self, tree: py_trees.behaviour.Behaviour, **kwargs) -> bool:
        """
        Setup each scenario before ticking

        Args:
            tree [py_trees.behavior.Behavior]: root of the tree

        return:
            True if the scenario is setup without errors
        """
        self.shutdown_requested = False
        self.blackboard = tree.attach_blackboard_client(
            name="MainBlackboardClient",
            namespace=tree.name
        )

        self.blackboard.register_key("end", access=py_trees.common.Access.READ)
        self.blackboard.register_key("fail", access=py_trees.common.Access.READ)

        # Initialize end and fail events
        self.blackboard.register_key("end", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key("fail", access=py_trees.common.Access.WRITE)
        self.blackboard.end = False
        self.blackboard.fail = False
        self.behaviour_tree = self.setup_behaviour_tree(tree)  # Get the behaviour_tree
        self.behaviour_tree.add_pre_tick_handler(self.pre_tick_handler)
        self.behaviour_tree.add_post_tick_handler(self.post_tick_handler)
        self.last_snapshot_visitor = LastSnapshotVisitor()
        self.behaviour_tree.add_visitor(self.last_snapshot_visitor)
        if self.debug:
            self.behaviour_tree.add_visitor(py_trees.visitors.DebugVisitor())
        if self.live_tree:
            self.behaviour_tree.add_visitor(
                py_trees.visitors.DisplaySnapshotVisitor(
                    display_blackboard=True
                ))
        try:
            self.behaviour_tree.setup(timeout=self.setup_timeout, logger=self.logger, output_dir=self.output_dir, **kwargs)
            return True
        except RuntimeError as e:
            self.logger.error(f'Runtime Error "{e}". Aborting... ')
            return False
        except Exception as e:  # pylint: disable=broad-except
            self.logger.error(f"Error while setting up tree: {e}")
            return False

    def setup_behaviour_tree(self, tree):
        """
        Setup the behaviour tree.

        For other middleware, a subclass of behaviour_tree might be needed for additional support.
        Override this to adapt to other middleware.

        Args:
            tree [py_trees.behaviour.Behaviour]: root of the behaviour tree

        return:
            py_trees.trees.BehaviourTree
        """
        return py_trees.trees.BehaviourTree(tree)

    def parse(self):
        """
        Parse the OpenScenario2 file

        return:
            True if no errors occured during parsing
        """
        if self.scenario_file is None:
            self.logger.error(f"No scenario file given.")
            return False
        file_extension = os.path.splitext(self.scenario_file)[1]
        if file_extension == '.osc':
            parser = OpenScenario2Parser(self.logger)
        elif file_extension == '.sce':
            parser = ModelFileLoader(self.logger)
        else:
            self.logger.error(f"File '{self.scenario_file}' has unknown extension '{file_extension}'. Allowed [.osc, .sce]")
            return False

        start = datetime.now()
        if not os.path.isfile(self.scenario_file):
            self.add_result(ScenarioResult(name=f'Parsing of {self.scenario_file}',
                                           result=False,
                                           failure_message="parsing failed",
                                           failure_output="File does not exist",
                                           processing_time=datetime.now() - start))
            return False
        self.scenarios = parser.process_file(self.scenario_file, self.log_model, self.debug)
        if self.scenarios is None:
            self.add_result(ScenarioResult(name=f'Parsing of {self.scenario_file}',
                                           result=False,
                                           failure_message="parsing failed",
                                           failure_output="No scenario defined",
                                           processing_time=datetime.now() - start))
            return False
        if len(self.scenarios) == 0:
            self.add_result(ScenarioResult(name=f'Parsing of {self.scenario_file}',
                                           result=False,
                                           failure_message="parsing failed",
                                           failure_output="no scenario defined",
                                           processing_time=datetime.now() - start))
        if len(self.scenarios) != 1:
            self.add_result(ScenarioResult(name=f'Parsing of {self.scenario_file}',
                                           result=False,
                                           failure_message="parsing failed",
                                           failure_output=f"more than one ({len(self.scenarios)}) scenario defined",
                                           processing_time=datetime.now() - start))

        return self.scenarios is not None and len(self.scenarios) == 1

    def run(self):
        if len(self.scenarios) != 1:
            self.logger.error(f"Only one scenario per file is supported.")
            return False
        self.current_scenario = self.scenarios[0]
        self.current_scenario_start = datetime.now()
        result = self.setup(self.current_scenario)
        if result:
            while not self.shutdown_requested:
                try:
                    self.behaviour_tree.tick()
                    time.sleep(self.tick_tock_period)
                    if self.live_tree:
                        self.logger.debug(py_trees.display.unicode_tree(
                            root=self.behaviour_tree.root, show_status=True))
                except KeyboardInterrupt:
                    self.on_scenario_shutdown(False, "Aborted")
        return self.process_results()

    def add_result(self, result: ScenarioResult):
        if result.result is False:
            self.logger.error(f"{result.name}: {result.failure_message} {result.failure_output}")
        self.results.append(result)

    def process_results(self):
        result = True
        if len(self.results) == 0:
            result = False
        else:
            for res in self.results:
                if res.result is False:
                    result = False

        # store output file
        if self.output_dir and self.results:
            result_file = os.path.join(self.output_dir, 'test.xml')
            # self.logger.info(f"Writing results to '{result_file}'...")
            failures = 0
            overall_time = timedelta(0)
            for res in self.results:
                if res.result is False:
                    failures += 1
                overall_time += res.processing_time
            try:
                with open(result_file, 'w') as out:
                    out.write('<?xml version="1.0" encoding="utf-8"?>\n')
                    out.write(
                        f'<testsuite errors="0" failures="{failures}" name="scenario_execution" tests="1" time="{overall_time.total_seconds()}">\n')
                    for res in self.results:
                        out.write(
                            f'  <testcase classname="tests.scenario" name="{res.name}" time="{res.processing_time.total_seconds()}">\n')
                        if res.result is False:
                            out.write(f'    <failure message="{res.failure_message}">{res.failure_output}</failure>\n')
                        out.write(f'  </testcase>\n')
                    out.write("</testsuite>\n")
            except Exception as e:  # pylint: disable=broad-except
                print(f"Could not write results to '{self.output_dir}': {e}")  # use print, as logger might not be available during shutdown
        return result

    def pre_tick_handler(self, behaviour_tree):
        """
        Things to do before a round of ticking
        """
        if self.live_tree:
            self.logger.debug(
                f"--------- Scenario {behaviour_tree.root.name}: Run {behaviour_tree.count} ---------")

    def post_tick_handler(self, behaviour_tree):
        # Shut down if the root has failed
        if self.behaviour_tree.root.status == py_trees.common.Status.FAILURE:
            self.blackboard.fail = True
        if self.behaviour_tree.root.status == py_trees.common.Status.SUCCESS:
            self.blackboard.end = True
        if self.blackboard.fail or self.blackboard.end:
            result = True
            if self.blackboard.fail:
                result = False
            if not self.shutdown_requested:
                self.on_scenario_shutdown(result)

    def on_scenario_shutdown(self, result, failure_message=""):
        self.shutdown_requested = True
        if self.behaviour_tree:
            self.behaviour_tree.interrupt()
        failure_output = ""
        if self.current_scenario:
            if result:
                self.logger.info(f"Scenario '{self.current_scenario.name}' succeeded.")
            else:
                if not failure_message:
                    failure_message = "execution failed."
                failure_output = self.last_snapshot_visitor.last_snapshot
                if self.log_model:
                    self.logger.error(self.last_snapshot_visitor.last_snapshot)
            self.add_result(ScenarioResult(name=self.current_scenario.name,
                                           result=result,
                                           failure_message=failure_message,
                                           failure_output=failure_output,
                                           processing_time=datetime.now()-self.current_scenario_start))

    @staticmethod
    def parse_args(args):
        parser = argparse.ArgumentParser()
        parser.add_argument('-d', '--debug', action='store_true', help='debugging output')
        parser.add_argument('-l', '--log-model', action='store_true',
                            help='Produce tree output of parsed openscenario2 content')
        parser.add_argument('-t', '--live-tree', action='store_true',
                            help='For debugging: Show current state of py tree')
        parser.add_argument('-o', '--output-dir', type=str, help='Directory for output (e.g. test results)')
        parser.add_argument('-n', '--dry-run', action='store_true', help='Parse and resolve scenario, but do not execute')
        parser.add_argument('scenario', type=str, help='scenario file to execute', nargs='?')
        args, _ = parser.parse_known_args(args)
        return args


def main():
    """
    main function
    """
    args = ScenarioExecution.parse_args(sys.argv[1:])
    try:
        scenario_execution = ScenarioExecution(debug=args.debug,
                                               log_model=args.log_model,
                                               live_tree=args.live_tree,
                                               scenario_file=args.scenario,
                                               output_dir=args.output_dir)
    except ValueError as e:
        print(f"Error while initializing: {e}")
        sys.exit(1)
    result = scenario_execution.parse()
    if result and not args.dry_run:
        result = scenario_execution.run()
    if result:
        sys.exit(0)
    else:
        sys.exit(1)


if __name__ == '__main__':
    main()
