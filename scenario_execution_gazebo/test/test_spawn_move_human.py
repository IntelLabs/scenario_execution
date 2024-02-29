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

""" Test for spawn and exists """
import os
import unittest
import psutil

from ament_index_python.packages import get_package_share_directory
import pytest
import launch_testing.actions

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


@pytest.mark.rostest
def generate_test_description():
    # pylint: disable=missing-function-docstring
    os.environ["PYTHONUNBUFFERED"] = '1'
    scenario_execution_gazebo_dir = get_package_share_directory('scenario_execution_gazebo')

    return (
        LaunchDescription([
            launch_testing.actions.ReadyToTest(),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(
                    scenario_execution_gazebo_dir, 'launch', 'test_ignition_scenario_launch.py')),
                launch_arguments={'headless': 'False',
                                  'debug': 'False',
                                  'scenario': os.path.join(scenario_execution_gazebo_dir,
                                                           'scenarios', 'test_spawn_walking_human.osc')}.items()),
        ]),
        {}
    )


class TestProcessOutput(unittest.TestCase):
    """
    Unit test for nav_to_pose
    """
    @unittest.skip(reason="requires porting")
    def test_output(self, proc_output, proc_info):
        # pylint: disable=missing-function-docstring
        proc_output.assertWaitFor("Executing scenario 'test_spawn_and_exists'", timeout=120)
        scenario_execution = None
        for prc in proc_info.process_names():
            if prc.startswith("scenario_execution"):
                scenario_execution = prc
                break
        assert scenario_execution
        proc_output.assertWaitFor("Scenario 'test_spawn_and_exists' succeeded.", timeout=120)

        # kill simulation before next test
        for proc in psutil.process_iter():
            if proc.name() == 'ruby':
                print(f"Kill gazebo process: {proc}")
                proc.kill()

        proc_info.assertWaitForShutdown(scenario_execution, timeout=120)
