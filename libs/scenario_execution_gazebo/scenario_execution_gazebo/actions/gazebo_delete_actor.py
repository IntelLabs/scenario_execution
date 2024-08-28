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


import py_trees
from enum import Enum

from scenario_execution.actions.run_process import RunProcess


class DeleteActionState(Enum):
    """
    States for executing a delete-entity in gazebo
    """
    IDLE = 1
    WAITING_FOR_RESPONSE = 2
    DONE = 3
    FAILURE = 4


class GazeboDeleteActor(RunProcess):
    """
    Class to delete an entity in gazebo

    """

    def __init__(self, associated_actor):
        super().__init__()
        self.entity_name = None
        self.current_state = DeleteActionState.IDLE

    def execute(self, associated_actor, entity_name: str, world_name: str):  # pylint: disable=arguments-differ
        self.set_command(["gz", "service", "-s", "/world/" + world_name + "/remove",
                          "--reqtype", "gz.msgs.Entity",
                          "--reptype", "gz.msgs.Boolean",
                          "--timeout", "1000", "--req", "name: \"" + entity_name + "\" type: MODEL"])

    def on_executed(self):
        """
        Hook when process gets executed
        """
        self.current_state = DeleteActionState.WAITING_FOR_RESPONSE

    def on_process_finished(self, ret):
        """
        check result of process

        return:
            py_trees.common.Status
        """
        if self.current_state == DeleteActionState.WAITING_FOR_RESPONSE:
            if ret == 0:
                while True:
                    try:
                        line = self.output.popleft()
                        line = line.lower()
                        if 'error' in line or 'timed out' in line:
                            self.feedback_message = f"Found error output while executing '{self.get_command()}'"  # pylint: disable= attribute-defined-outside-init
                            self.current_state = DeleteActionState.FAILURE
                            return py_trees.common.Status.FAILURE
                    except IndexError:
                        break
                self.current_state = DeleteActionState.DONE
                return py_trees.common.Status.SUCCESS
            else:
                self.current_state = DeleteActionState.FAILURE
                return py_trees.common.Status.FAILURE
        else:
            return py_trees.common.Status.INVALID
