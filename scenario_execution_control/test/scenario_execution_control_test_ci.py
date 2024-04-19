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

import subprocess  # nosec B404

node_process = subprocess.Popen(['ros2', 'launch', 'scenario_execution_control', 'scenario_execution_control_launch.py',
                                'scenario_dir:=scenario_execution_control/test/scenarios/', 'output_dir:=test_example_control'], stdout=subprocess.PIPE, stderr=subprocess.PIPE)

test_service_call = subprocess.run(['python3', 'scenario_execution_control/test/scenario_execution_control_test.py',
                                   'scenario_execution_control/test/scenarios/scenario_execution_control_test.osc'], check=True)

exit_status = test_service_call.returncode

if exit_status == 0:
    print('Test passed successfully.')
else:
    print("Test failed")

node_process.terminate()
node_process.wait()
