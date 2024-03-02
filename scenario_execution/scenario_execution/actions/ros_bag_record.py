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

""" Class for recording a ros bag """

import os
from datetime import datetime
from enum import Enum

import py_trees
from scenario_execution_base.actions import RunExternalProcess
import shutil
import signal


class RosBagRecordActionState(Enum):
    """
    States for executing a ros bag recording
    """
    WAITING_FOR_TOPICS = 1
    RECORDING = 2
    FAILURE = 5


class RosBagRecord(RunExternalProcess):
    """
    Class to execute ros bag recording
    """

    def __init__(self, name, destination_dir: str, topics: list, timestamp_suffix: bool, hidden_topics: bool, storage: str):
        super().__init__(name)
        if not isinstance(topics, list):
            raise TypeError(f'Topics needs to be list of topics, got {type(topics)}.')
        else:
            self.topics = topics
        self.destination_dir = destination_dir
        self.timestamp_suffix = timestamp_suffix
        self.include_hidden_topics = hidden_topics
        self.current_state = RosBagRecordActionState.WAITING_FOR_TOPICS
        self.bag_dir = ''
        self.storage = storage
        self.command = None

    def setup(self, **kwargs):
        """
        set up
        """
        if self.destination_dir:
            if not os.path.exists(self.destination_dir):
                raise ValueError(
                    f"Specified destination dir '{self.destination_dir}' does not exist")
            self.bag_dir = self.destination_dir + '/'
        self.bag_dir += self.get_scenario_name()

        if self.timestamp_suffix:
            self.bag_dir += '_' + datetime.now().strftime("%Y%m%d%H%M%S")

        self.command = ["ros2", "bag", "record"]
        if self.include_hidden_topics:
            self.command.append("--include-hidden-topics")
        if self.storage:
            self.command.extend(["--storage", self.storage])

        self.command.extend(["-o", self.bag_dir] + self.topics)

        self.logger.info(f'Command: {" ".join(self.command)}')

    def get_scenario_name(self):
        """
        get scenario name from py tree root
        """
        parent = self.parent
        scenario_name = "INVALID"
        while parent:
            scenario_name = parent.name
            parent = parent.parent
        return scenario_name

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

    def cleanup(self):
        """
        Cleanup on shutdown
        """
        if self.current_state != RosBagRecordActionState.FAILURE:
            self.logger.info('Waiting for process to quit...')
            if self.process:
                self.process.send_signal(signal.SIGINT)
                self.process.wait()
            self.logger.info('Process finished.')
        if self.current_state == RosBagRecordActionState.WAITING_FOR_TOPICS and self.bag_dir and os.path.exists(self.bag_dir):
            self.logger.info(
                f'Cleanup while waiting for topics. Removing incomplete bag {self.bag_dir}...')
            shutil.rmtree(self.bag_dir)

    def on_process_finished(self, ret):
        """
        check result of process

        return:
            py_trees.common.Status
        """
        if self.current_state == RosBagRecordActionState.RECORDING:
            self.current_state = RosBagRecordActionState.FAILURE
        return py_trees.common.Status.FAILURE
