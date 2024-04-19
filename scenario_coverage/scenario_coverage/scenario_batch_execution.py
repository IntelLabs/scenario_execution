#! /usr/bin/env python3

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
import argparse
import subprocess  # nosec B404
from threading import Thread
from copy import deepcopy
import signal
from defusedxml import ElementTree as ETparse
import xml.etree.ElementTree as ET  # nosec B405
import logging


class ScenarioBatchExecution(object):

    def __init__(self, args) -> None:
        if not os.path.isdir(args.output_dir):
            try:
                os.mkdir(args.output_dir)
            except OSError as e:
                raise ValueError(f"Could not create output directory: {e}") from e
        if not os.access(args.output_dir, os.W_OK):
            raise ValueError(f"Output directory '{args.output_dir}' not writable.")
        if os.path.exists(os.path.join(args.output_dir, 'test.xml')):
            os.remove(os.path.join(args.output_dir, 'test.xml'))
        self.output_dir = args.output_dir

        dir_content = os.listdir(args.scenario_dir)
        self.scenarios = []
        for entry in dir_content:
            if entry.endswith(".sce") or entry.endswith(".osc"):
                self.scenarios.append(os.path.join(args.scenario_dir, entry))
        if not self.scenarios:
            raise ValueError(f"Directory {args.scenario_dir} does not contain any scenarios.")
        self.scenarios.sort()
        print(f"Detected {len(self.scenarios)} scenarios.")
        self.launch_command = args.launch_command
        if self.get_launch_command("", "") is None:
            raise ValueError("Launch command does not contain {SCENARIO} and {OUTPUT_DIR}: " + " ".join(args.launch_command))
        print(f"Launch command: {self.launch_command}")

    def get_launch_command(self, scenario_name, output_dir):
        launch_command = deepcopy(self.launch_command)
        scenario_replaced = False
        output_dir_replaced = False
        for i in range(0, len(launch_command)):  # pylint: disable=consider-using-enumerate
            if "{SCENARIO}" in launch_command[i]:
                launch_command[i] = launch_command[i].replace('{SCENARIO}', scenario_name)
                scenario_replaced = True
            if "{OUTPUT_DIR}" in launch_command[i]:
                launch_command[i] = launch_command[i].replace('{OUTPUT_DIR}', output_dir)
                output_dir_replaced = True
        if scenario_replaced and output_dir_replaced:
            return launch_command
        else:
            return None

    def run(self) -> bool:
        def log_output(out, logger):
            try:
                for line in iter(out.readline, b''):
                    msg = line.decode().strip()
                    print(msg)
                    logger.info(msg)
                out.close()
            except ValueError:
                pass

        def configure_logger(log_file_path):
            logger = logging.getLogger(log_file_path)
            if logger.hasHandlers():
                logger.handlers.clear()
            file_handler = logging.FileHandler(filename=log_file_path, mode='a')
            file_handler.setFormatter(logging.Formatter('%(message)s'))
            file_handler.setLevel(logging.INFO)
            logger.addHandler(file_handler)
            logger.setLevel(logging.INFO)
            return logger

        ret = True
        for scenario in self.scenarios:
            scenario_name = os.path.splitext(os.path.basename(scenario))[0]
            output_file_path = os.path.join(self.output_dir, scenario_name)
            if not os.path.isdir(output_file_path):
                os.mkdir(output_file_path)
            launch_command = self.get_launch_command(scenario, output_file_path)
            log_cmd = " ".join(launch_command)
            print(f"### For scenario {scenario}, executing process: '{log_cmd}'")
            process = subprocess.Popen(launch_command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, stdin=subprocess.PIPE)
            file_handler = logging.FileHandler(filename=os.path.join(output_file_path, scenario_name + '.log'), mode='w')
            logger = configure_logger(os.path.join(output_file_path, scenario_name + '.log'))
            log_stdout_thread = Thread(target=log_output, args=(process.stdout, logger, ))
            log_stdout_thread.daemon = True  # die with the program
            log_stdout_thread.start()
            log_stderr_thread = Thread(target=log_output, args=(process.stderr, logger, ))
            log_stderr_thread.daemon = True  # die with the program
            log_stderr_thread.start()

            print(f"### Waiting for process to finish...")
            try:
                process.wait()
                if process.returncode:
                    print("### Process failed.")
                    ret = False
                else:
                    print("### Process finished successfully.")
            except KeyboardInterrupt:
                print("### Interrupted by user. Sending SIGINT...")
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=20)
                    return False
                except subprocess.TimeoutExpired:
                    print("### Process not stopped after 20s. Sending SIGKILL...")
                    process.send_signal(signal.SIGKILL)
                try:
                    process.wait(timeout=10)
                    return False
                except subprocess.TimeoutExpired:
                    print("### Process not stopped after 10s.")
                return False
            file_handler.flush()
            file_handler.close()
        xml_ret = self.combine_test_xml()
        return xml_ret and ret

    def combine_test_xml(self):
        print(f"### Writing combined tests to '{self.output_dir}/test.xml'.....")
        tree = ET.Element('testsuite')
        total_time = 0
        total_errors = 0
        total_failures = 0
        total_tests = 0
        for scenario in self.scenarios:
            scenario_name = os.path.splitext(os.path.basename(scenario))[0]
            test_file = os.path.join(self.output_dir, scenario_name, 'test.xml')
            parsed_successfully = False
            if os.path.exists(test_file):
                try:
                    test_tree = ETparse.parse(test_file)
                    root = test_tree.getroot()
                except ETparse.ParseError:
                    print(f"### Error XML file {test_file} could not be parsed")
                if root is not None:
                    parsed_successfully = True
                    total_errors += int(root.attrib.get('errors', 0))
                    total_failures += int(root.attrib.get('failures', 0))
                    total_time += float(root.attrib.get('time', 0))
                    total_tests += int(root.attrib.get('tests', 0))
                    for testcase in root.findall('testcase'):
                        testcase.set('name', str(scenario_name))
                        tree.append(testcase)
                else:
                    print(f"### XML file has no 'testsuite' element. {test_file}")

            if not parsed_successfully:
                missing_test_elem = ET.Element('testcase')
                missing_test_elem.set("classname", "tests.scenario")
                missing_test_elem.set("name", "no_test_result")
                missing_test_elem.set("time", "0.0")
                failure_elem = ET.Element('failure')
                failure_elem.set("message", f"expected file {test_file} not found")
                missing_test_elem.append(failure_elem)
                tree.append(missing_test_elem)
        tree.set('errors', str(total_errors))
        tree.set('failures', str(total_failures))
        tree.set('time', str(total_time))
        tree.set('tests', str(total_tests))
        combined_tests = ET.ElementTree(tree)
        ET.indent(combined_tests, space="\t", level=0)
        combined_tests.write(os.path.join(self.output_dir, "test.xml"), encoding='utf-8', xml_declaration=True)
        return total_errors == 0 and total_failures == 0


def main():
    """
    main function
    """
    parser = argparse.ArgumentParser()
    parser.add_argument('-i', '--scenario-dir', type=str, help='Directory containing the scenarios')
    parser.add_argument('-o', '--output-dir', type=str, help='Directory containing the output', default='out')
    parser.add_argument('launch_command', nargs='+')
    args = parser.parse_args(sys.argv[1:])

    try:
        scenario_batch_execution = ScenarioBatchExecution(args)
    except Exception as e:  # pylint: disable=broad-except
        print(f"Error while initializing batch execution: {e}")
        sys.exit(1)
    if scenario_batch_execution.run():
        sys.exit(0)
    else:
        print("Error during batch executing!")
        sys.exit(1)


if __name__ == '__main__':
    main()
