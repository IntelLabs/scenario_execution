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

from enum import Enum
import py_trees
import os
from scenario_execution.actions.base_action import ActionError
from scenario_execution.actions.run_process import RunProcess


class CaptureScreenState(Enum):
    IDLE = 1
    CAPTURING = 2
    DONE = 11
    FAILURE = 12


class CaptureScreen(RunProcess):

    def __init__(self):
        super().__init__()
        self.current_state = None
        self.output_dir = "."

    def setup(self, **kwargs):
        if "DISPLAY" not in os.environ:
            raise ActionError("capture_screen() requires environment variable 'DISPLAY' to be set.", action=self)

        if kwargs['output_dir']:
            if not os.path.exists(kwargs['output_dir']):
                raise ActionError(
                    f"Specified destination dir '{kwargs['output_dir']}' does not exist", action=self)
            self.output_dir = kwargs['output_dir']

    def execute(self, output_filename: str, frame_rate: float):  # pylint: disable=arguments-differ
        super().execute(None, wait_for_shutdown=True)
        self.current_state = CaptureScreenState.IDLE
        cmd = ["ffmpeg",
               "-f", "x11grab",
               "-draw_mouse", "0",
               "-framerate", str(frame_rate),
               "-i", os.environ["DISPLAY"],
               "-c:v", "libx264",
               "-preset", "veryfast",
               "-f", "mp4",
               "-nostdin",
               "-y", os.path.join(self.output_dir, output_filename)]
        self.set_command(cmd)

    def get_logger_stdout(self):
        return self.logger.debug

    def get_logger_stderr(self):
        return self.logger.debug

    def on_executed(self):
        self.current_state = CaptureScreenState.CAPTURING
        self.feedback_message = f"Capturing screen..."  # pylint: disable= attribute-defined-outside-init

    def on_process_finished(self, ret):
        if self.current_state == CaptureScreenState.CAPTURING:
            self.feedback_message = f"Capturing screen failed. {self.output[-1]}"  # pylint: disable= attribute-defined-outside-init
            return py_trees.common.Status.FAILURE
        return py_trees.common.Status.SUCCESS
