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
from datetime import datetime
from enum import Enum

import py_trees
from scenario_execution.actions.base_action import ActionError
from scenario_execution.actions.run_process import RunProcess
import shutil
import signal


class RosBagRecordActionState(Enum):
    """
    States for executing a ros bag recording
    """
    WAITING_FOR_TOPICS = 1
    RECORDING = 2
    FAILURE = 5


class RosBagRecord(RunProcess):
    """
    Class to execute ros bag recording
    """

    def __init__(self):
        super().__init__()
        self.bag_dir = None
        self.current_state = RosBagRecordActionState.WAITING_FOR_TOPICS
        self.command = None
        self.output_dir = None
        self.topics = None

    def setup(self, **kwargs):
        """
        set up
        """
        if "output_dir" not in kwargs:
            raise ActionError("output_dir not defined.", action=self)

        if kwargs['output_dir']:
            if not os.path.exists(kwargs['output_dir']):
                raise ActionError(f"Specified destination dir '{kwargs['output_dir']}' does not exist", action=self)
            self.output_dir = kwargs['output_dir']

    def execute(self, topics: list, timestamp_suffix: bool, hidden_topics: bool, storage: str, use_sim_time: bool):  # pylint: disable=arguments-differ
        self.bag_dir = ''
        if self.output_dir:
            self.bag_dir = self.output_dir + '/'
        self.bag_dir += "rosbag2"

        if timestamp_suffix:
            self.bag_dir += '_' + datetime.now().strftime("%Y_%m_%d-%H_%M_%S")

        self.topics = topics
        self.command = ["ros2", "bag", "record"]
        if hidden_topics:
            self.command.append("--include-hidden-topics")
        if storage:
            self.command.extend(["--storage", storage])
        if use_sim_time:
            self.command.append("--use-sim-time")
        self.command.extend(["-o", self.bag_dir] + self.topics)

    def get_logger_stderr(self):
        """
        get logger for stderr messages
        """
        return self.logger.info  # ros2 bag record reports all messages on stderr

    def on_executed(self):
        """
        Hook when process gets executed
        """
        self.feedback_message = f"Waiting for all topics to subscribe..."  # pylint: disable= attribute-defined-outside-init
        self.current_state = RosBagRecordActionState.WAITING_FOR_TOPICS

    def check_running_process(self):
        """
        hook to check running process

        return:
            py_trees.common.Status
        """
        if self.current_state == RosBagRecordActionState.WAITING_FOR_TOPICS:
            while True:
                try:
                    line = self.output.popleft()
                    if line.endswith('All requested topics are subscribed. Stopping discovery...'):
                        self.feedback_message = f"Recording..."  # pylint: disable= attribute-defined-outside-init
                        self.current_state = RosBagRecordActionState.RECORDING
                        return py_trees.common.Status.SUCCESS
                except IndexError:
                    break
            return py_trees.common.Status.RUNNING
        else:
            self.current_state = RosBagRecordActionState.FAILURE
            return py_trees.common.Status.FAILURE

    def shutdown(self):
        if self.current_state != RosBagRecordActionState.FAILURE:
            self.logger.info('Waiting for process to quit...')
            if self.process:
                self.process.send_signal(signal.SIGINT)
                self.process.wait()
            self.logger.info('Process finished.')
        if self.current_state == RosBagRecordActionState.WAITING_FOR_TOPICS and self.bag_dir and os.path.exists(self.bag_dir):
            self.logger.info(
                f'Shutdown while waiting for topics. Removing incomplete bag {self.bag_dir}...')
            shutil.rmtree(self.bag_dir)

    def on_process_finished(self, ret):
        """
        check result of process

        return:
            py_trees.common.Status
        """
        if self.current_state == RosBagRecordActionState.WAITING_FOR_TOPICS:
            if ret > 0:
                line = None
                while True:
                    try:
                        line = self.output.popleft()
                    except IndexError:
                        break
                if line:
                    self.feedback_message = f"{line}"  # pylint: disable= attribute-defined-outside-init
                else:
                    self.feedback_message = f"Error while executing ros2 bag record."  # pylint: disable= attribute-defined-outside-init
        if self.current_state == RosBagRecordActionState.RECORDING:
            self.current_state = RosBagRecordActionState.FAILURE
        return py_trees.common.Status.FAILURE
