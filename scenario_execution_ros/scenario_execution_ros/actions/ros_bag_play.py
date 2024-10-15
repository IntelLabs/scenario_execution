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
import py_trees
from scenario_execution.actions.base_action import ActionError
from scenario_execution.actions.run_process import RunProcess


class RosBagPlay(RunProcess):
    """
    Class to execute ros bag recording
    """

    def __init__(self):
        super().__init__()
        self.input_dir = None
        self.source = None

    def setup(self, **kwargs):
        if "input_dir" not in kwargs:
            raise ActionError("input_dir not defined.", action=self)
        self.input_dir = kwargs['input_dir']

    def execute(self, source: str, topics: list, publish_clock: bool, publish_clock_rate: float, start_offset: float):  # pylint: disable=arguments-differ,arguments-renamed
        super().execute(wait_for_shutdown=True)
        self.source = source
        bag_dir = ''
        if os.path.isabs(source):
            bag_dir = source
        else:
            bag_dir = os.path.join(self.input_dir, source)
        if not os.path.exists(bag_dir):
            raise ActionError(f"Specified rosbag directory '{bag_dir}' does not exist", action=self)

        self.command = ["ros2", "bag", "play", "--disable-keyboard-controls"]
        if publish_clock:
            self.command.extend(["--clock", str(publish_clock_rate)])
        if start_offset:
            self.command.extend(["--start-offset", str(start_offset)])
        if topics:
            topics_string = " ".join(topics)
            self.command.append(f"--topics '{topics_string}'")
        self.command.append(bag_dir)

    def get_logger_stderr(self):
        return self.logger.info  # ros2 bag play reports all messages on stderr

    def on_executed(self):
        self.feedback_message = f"Playing back {self.source}..."  # pylint: disable= attribute-defined-outside-init

    def on_process_finished(self, ret):
        if ret == 0:
            self.feedback_message = f"Playback of {self.source} finished."  # pylint: disable= attribute-defined-outside-init
            return py_trees.common.Status.SUCCESS
        else:
            self.feedback_message = f"Playback of '{self.source}' failed with {ret}"  # pylint: disable= attribute-defined-outside-init
            return py_trees.common.Status.FAILURE
