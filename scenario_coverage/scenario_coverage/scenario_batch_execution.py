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
from collections import deque
from copy import deepcopy
import signal
from defusedxml import ElementTree as ETparse
defusedxml.defuse_stdlib()
import xml.etree.ElementTree as ET


class ScenarioBatchExecution(object):

    def __init__(self, args) -> None:
        if not os.path.isdir(args.output_dir):
            os.mkdir(args.output_dir)
        self.output_dir = args.output_dir

        dir_content = os.listdir(args.scenario_dir)
        self.scenarios = []
        for entry in dir_content:
            if entry.endswith(".sce") or entry.endswith(".osc"):
                self.scenarios.append(os.path.join(args.scenario_dir, entry))
        if not self.scenarios:
            raise ValueError(f"Directory {args.scenario_dir} does not contain any scenarios.")
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

        def log_output(out, buffer):
            try:
                for line in iter(out.readline, b''):
                    msg = line.decode().strip()
                    print(msg)
                    buffer.append(msg)
                out.close()
            except ValueError:
                pass

        for scenario in self.scenarios:
            output_file_path = os.path.join(self.output_dir, os.path.splitext(os.path.basename(scenario))[0])
            if not os.path.isdir(output_file_path):
                os.mkdir(output_file_path)
            launch_command = self.get_launch_command(scenario, output_file_path)
            output = deque()
            log_cmd = " ".join(launch_command)
            print(f"### For scenario {scenario}, executing process: '{log_cmd}'")
            process = subprocess.Popen(launch_command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, stdin=subprocess.PIPE)

            log_stdout_thread = Thread(target=log_output, args=(process.stdout, output, ))
            log_stdout_thread.daemon = True  # die with the program
            log_stdout_thread.start()

            log_stderr_thread = Thread(target=log_output, args=(process.stderr, output, ))
            log_stderr_thread.daemon = True  # die with the program
            log_stderr_thread.start()

            print(f"### Waiting for process to finish...")
            try:
                process.wait()
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
            ret = process.returncode

            print(f"### Storing results in {self.output_dir}...")

            with open(output_file_path + '.log', 'w') as out:
                for line in output:
                    out.write(line + '\n')
            if ret:
                print("### Process failed.")
            else:
                print("### Process finished successfully.")
        self.combine_test_xml()
        return True

    def combine_test_xml(self):
        print(f"### Writing combined tests to '{self.output_dir}/test.xml'.....")
        new_root = ET.Element('testsuite')
        total_time = 0
        total_errors = 0
        total_failures = 0
        total_tests = 0
        for scenario in self.scenarios:
            variation = os.path.splitext(os.path.basename(scenario))[0]
            for filename in os.listdir(os.path.join(self.output_dir, variation)):
                if filename.endswith("test.xml"):
                    file_path = os.path.join(self.output_dir, variation, filename)
                    try:
                        tree = ETparse.parse(file_path)
                        root = tree.getroot()
                    except FileNotFoundError:
                        print(f"### File {file_path} not found")
                        return
                    except ETparse.ParseError:
                        print(f"### Error XML file {file_path} could not be parsed")
                        return
                    if root is not None:
                        total_errors += int(root.attrib.get('errors', 0))
                        total_failures += int(root.attrib.get('failures', 0))
                        total_time += float(root.attrib.get('time', 0))
                        total_tests += int(root.attrib.get('tests', 0))
                        for testcase in root.findall('testcase'):
                            testcase.set('name', str(variation))
                            new_root.append(testcase)
                    else:
                        print(f"### XML file has no 'testsuite' element. {file_path}")
        new_root.set('errors', str(total_errors))
        new_root.set('failures', str(total_failures))
        new_root.set('time', str(total_time))
        new_root.set('tests', str(total_tests))
        combined_tests = ET.ElementTree(new_root)
        ET.indent(combined_tests, space="\t", level=0)
        combined_tests.write(os.path.join(self.output_dir, "test.xml"), encoding='utf-8', xml_declaration=True)


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
