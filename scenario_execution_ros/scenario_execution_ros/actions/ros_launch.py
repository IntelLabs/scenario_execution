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
from enum import Enum

import py_trees
from scenario_execution.actions.run_process import RunProcess
import shutil
import signal


class RosLaunchActionState(Enum):
    """
    States for executing a ros bag recording
    """
    WAITING_FOR_TOPICS = 1
    RECORDING = 2
    FAILURE = 5


class RosLaunch(RunProcess):
    """
    Class to execute ros bag recording
    """

    def __init__(self, name, package_name: str, launch_file: str, arguments: list, wait_for_finish: bool):
        super().__init__(name)
        self.package_name = package_name
        self.launch_file = launch_file
        self.arguments = arguments
        self.wait_for_finish = wait_for_finish
        # if not isinstance(topics, list):
        #     raise TypeError(f'Topics needs to be list of topics, got {type(topics)}.')
        # else:
        #     self.topics = topics
        # self.timestamp_suffix = timestamp_suffix
        # self.include_hidden_topics = hidden_topics
        # self.current_state = RosLaunchActionState.WAITING_FOR_TOPICS
        # self.bag_dir = ''
        # self.storage = storage
        self.command = None

    def setup(self, **kwargs):
        """
        set up
        """
        # if "output_dir" not in kwargs:
        #     raise ValueError("output_dir not defined.")

        # if kwargs['output_dir']:
        #     if not os.path.exists(kwargs['output_dir']):
        #         raise ValueError(
        #             f"Specified destination dir '{kwargs['output_dir']}' does not exist")
        #     self.bag_dir = kwargs['output_dir'] + '/'
        # self.bag_dir += "rosbag2"

        # if self.timestamp_suffix:
        #     self.bag_dir += '_' + datetime.now().strftime("%Y_%m_%d-%H_%M_%S")

        self.command = ["ros2", "launch", self.package_name, self.launch_file]

        for arg in self.arguments:
            if not arg["key"] or not arg["value"]:
                raise ValueError(f'Invalid ros argument key:{arg["key"]}, value:{arg["value"]}')
            self.command.append(f'{arg["key"]}:={arg["value"]}')

        self.logger.info(f'Command: {" ".join(self.command)}')

    # def get_scenario_name(self):
    #     """
    #     get scenario name from py tree root
    #     """
    #     parent = self.parent
    #     scenario_name = "INVALID"
    #     while parent:
    #         scenario_name = parent.name
    #         parent = parent.parent
    #     return scenario_name

    def get_logger_stderr(self):
        """
        get logger for stderr messages
        """
        return self.logger.info  # ros2 bag record reports all messages on stderr

    # def on_executed(self):
    #     """
    #     Hook when process gets executed
    #     """
    #     self.feedback_message = f"Waiting for all topics to subscribe..."  # pylint: disable= attribute-defined-outside-init
    #     self.current_state = RosLaunchActionState.WAITING_FOR_TOPICS

    # def check_running_process(self):
    #     """
    #     hook to check running process

    #     return:
    #         py_trees.common.Status
    #     """
    #     if self.current_state == RosLaunchActionState.WAITING_FOR_TOPICS:
    #         while True:
    #             try:
    #                 line = self.output.popleft()
    #                 if line.endswith('All requested topics are subscribed. Stopping discovery...'):
    #                     self.feedback_message = f"Recording..."  # pylint: disable= attribute-defined-outside-init
    #                     self.current_state = RosLaunchActionState.RECORDING
    #                     return py_trees.common.Status.SUCCESS
    #             except IndexError:
    #                 break
    #         return py_trees.common.Status.RUNNING
    #     else:
    #         self.current_state = RosLaunchActionState.FAILURE
    #         return py_trees.common.Status.FAILURE

    def shutdown(self):
        if self.current_state != RosLaunchActionState.FAILURE:
            self.logger.info('Waiting for process to quit...')
            if self.process:
                self.process.send_signal(signal.SIGINT)
                self.process.wait()
            self.logger.info('Process finished.')
        if self.current_state == RosLaunchActionState.WAITING_FOR_TOPICS and self.bag_dir and os.path.exists(self.bag_dir):
            self.logger.info(
                f'Shutdown while waiting for topics. Removing incomplete bag {self.bag_dir}...')

    # def on_process_finished(self, ret):
    #     """
    #     check result of process

    #     return:
    #         py_trees.common.Status
    #     """
    #     if self.current_state == RosLaunchActionState.RECORDING:
    #         self.current_state = RosLaunchActionState.FAILURE
    #     return py_trees.common.Status.FAILURE
