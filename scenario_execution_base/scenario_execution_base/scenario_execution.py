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
from datetime import datetime, timedelta
import py_trees
from scenario_execution_base.model.osc2_parser import OpenScenario2Parser
from scenario_execution_base.utils.logging import Logger
from scenario_execution_base.model.model_file_loader import ModelFileLoader


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
                 scenario: str,
                 output_dir: str,
                 setup_timeout=py_trees.common.Duration.INFINITE,
                 tick_tock_period: float = 0.1) -> None:
        self.debug = debug
        self.log_model = log_model
        self.live_tree = live_tree
        self.scenario = scenario
        self.output_dir = output_dir
        self.logger = self._get_logger()

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

    def _get_logger(self):
        """
        Create a logger
        This method could be overriden by child classes and defined according to the middleware.

        return:
            A logger which has three logging levels: info, warning, error
        """
        return Logger('scenario_execution')

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
        except RuntimeError:
            self.logger.error('Setup Timeout exceeded. Aborting...')
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
        file_extension = os.path.splitext(self.scenario)[1]
        if file_extension == '.osc':
            parser = OpenScenario2Parser(self.logger)
        elif file_extension == '.sce':
            parser = ModelFileLoader(self.logger)
        else:
            self.logger.error(f"File '{self.scenario}' has unknown extension '{file_extension}'. Allowed [.osc, .sce]")
            return False

        start = datetime.now()
        self.scenarios = parser.process_file(self.scenario, self.log_model, self.debug)
        if self.scenarios is None:
            self.add_result((f'Parsing of {self.scenario}', True, "parsing failed", "", datetime.now() - start))
        return self.scenarios is not None

    def run(self) -> bool:
        """
        Run all scenarios

        return:
            True if all scenarios are executed successfully
        """
        if not self.scenarios:
            self.logger.info("No scenarios to execute.")

        failure = False
        for tree in self.scenarios:
            start = datetime.now()
            if not tree:
                self.logger.error(f'Scenario {tree.name} has no executables.')
                continue
            if not self.setup(tree):
                failure = True
                self.logger.error(f'Scenario {tree.name} failed to setup.')
                continue
            while not self.shutdown_requested:
                try:
                    self.behaviour_tree.tick()
                    time.sleep(self.tick_tock_period)
                    if self.live_tree:
                        self.logger.debug(py_trees.display.unicode_tree(
                            root=self.behaviour_tree.root, show_status=True))
                except KeyboardInterrupt:
                    self.behaviour_tree.interrupt()
                    self.blackboard.fail = True
                    break
            if self.blackboard.fail:
                self.logger.error(f'Scenario {tree.name} failed.')
            failure = failure or self.blackboard.fail
            self.add_result((tree.name, self.blackboard.fail, "execution failed", "", datetime.now()-start))
            self.cleanup_behaviours(tree)
        return not failure

    def add_result(self, result):
        self.results.append(result)

    def report_results(self):
        if self.output_dir and self.results:
<<<<<<< Updated upstream
            self.logger.info(f"Writing results to {self.output_dir}...")
=======
            self.logger.info(f"Writing results to '{self.output_dir}'...")
>>>>>>> Stashed changes
            failures = 0
            overall_time = timedelta(0)
            for result in self.results:
                if result[1]:
                    failures += 1
                overall_time += result[4]
            try:
<<<<<<< Updated upstream
                with open(self.output_dir, 'w') as out:
=======
                with open(os.path.join(self.output_dir, 'test.xml'), 'w') as out:
>>>>>>> Stashed changes
                    out.write('<?xml version="1.0" encoding="utf-8"?>\n')
                    out.write(
                        f'<testsuite errors="0" failures="{failures}" name="scenario_execution" tests="1" time="{overall_time.total_seconds()}">\n')
                    for result in self.results:
                        out.write(f'  <testcase classname="tests.scenario" name="{result[0]}" time="{result[4].total_seconds()}">\n')
                        if result[1]:
                            out.write(f'    <failure message="{result[2]}">{result[3]}</failure>\n')
                        out.write(f'  </testcase>\n')
                    out.write("</testsuite>\n")
            except Exception as e:  # pylint: disable=broad-except
<<<<<<< Updated upstream
                self.logger.error(f"Could not write results to {self.output_dir}: {e}")
=======
                self.logger.error(f"Could not write results to '{self.output_dir}': {e}")
>>>>>>> Stashed changes

    def pre_tick_handler(self, behaviour_tree):
        """
        Things to do before a round of ticking
        """
        if self.live_tree:
            self.logger.debug(
                f"--------- Scenario {behaviour_tree.root.name}: Run {behaviour_tree.count} ---------")

    def post_tick_handler(self, behaviour_tree):
        """
        Things to do after a round of ticking
        """
        # Shut down if the root is failed
        if self.behaviour_tree.root.status == py_trees.common.Status.FAILURE:
            self.blackboard.fail = True
        if self.behaviour_tree.root.status == py_trees.common.Status.SUCCESS:
            self.blackboard.end = True
        self.shutdown_requested = self.blackboard.fail or self.blackboard.end

    def cleanup_behaviours(self, tree):
        """
        Run cleanup functions in all behaviors
        """
        class CleanupVisitor(py_trees.visitors.VisitorBase):
            """
            Helper class to call cleanup functions in all behaviors
            """

            def __init__(self):
                super(CleanupVisitor, self).__init__(full=False)

            def run(self, behaviour):
                """
                call cleanup method
                """
                method = getattr(behaviour, 'cleanup', None)
                if callable(method):
                    method()

        cleanup_visitor = CleanupVisitor()

        for node in tree.iterate():
            cleanup_visitor.run(node)

    @staticmethod
    def parse_args(args):
        parser = argparse.ArgumentParser()
        parser.add_argument('-d', '--debug', action='store_true', help='debugging output')
        parser.add_argument('-l', '--log-model', action='store_true',
                            help='Produce tree output of parsed openscenario2 content')
        parser.add_argument('-t', '--live-tree', action='store_true',
                            help='For debugging: Show current state of py tree')
        parser.add_argument('-o', '--output-dir', type=str, help='Directory for output (e.g. test results)')
        parser.add_argument('scenario', type=str, help='scenario file to execute', nargs='?')
        args = parser.parse_args(args)
        return args


def main():
    """
    main function
    """
    args = ScenarioExecution.parse_args(sys.argv[1:])
    scenario_execution = ScenarioExecution(debug=args.debug,
                                           log_model=args.log_model,
                                           live_tree=args.live_tree,
                                           scenario=args.scenario,
                                           output_dir=args.output_dir)

    result = scenario_execution.parse()
    if result:
        result = scenario_execution.run()
    scenario_execution.report_results()
    if result:
        sys.exit(0)
    else:
        sys.exit(1)


if __name__ == '__main__':
    main()
