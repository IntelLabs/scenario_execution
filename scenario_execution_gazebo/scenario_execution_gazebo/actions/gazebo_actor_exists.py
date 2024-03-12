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

from scenario_execution_base.actions.run_process import RunProcess

import py_trees
from enum import Enum
import json


class ActorExistsActionState(Enum):
    """
    States for executing a entity check in ignition
    """
    IDLE = 1
    WAITING_FOR_ACTOR = 2
    DONE = 3
    FAILURE = 4


class GazeboActorExists(RunProcess):
    """
    Class to check existance of an entity in Ignition

    """

    def __init__(self, entity_name: str, world_name: str):
        """
        init
        """
        super().__init__('GazeboActorExistsAction')
        self.entity_name = entity_name
        self.node = None
        self.set_command(["ign", "topic", "-t", "/world/" +
                          world_name + "/pose/info", "-e", "--json-output"])
        self.current_state = ActorExistsActionState.IDLE

    def on_executed(self):
        """
        Hook when process gets executed
        """
        self.feedback_message = f"Waiting for entity '{self.entity_name}'"  # pylint: disable= attribute-defined-outside-init
        self.current_state = ActorExistsActionState.WAITING_FOR_ACTOR

    def get_logger_stdout(self):
        """
        get logger for stderr messages
        """
        return None

    def check_running_process(self):
        """
        hook to check running process

        return:
            py_trees.common.Status
        """
        if self.current_state == ActorExistsActionState.WAITING_FOR_ACTOR:
            while True:
                try:
                    line = self.output.popleft()
                    pose_state = json.loads(line)
                    for pose in pose_state['pose']:
                        if pose['name'] == self.entity_name:
                            self.feedback_message = f"Found entity '{self.entity_name}'"  # pylint: disable= attribute-defined-outside-init
                            self.current_state = ActorExistsActionState.DONE
                            return py_trees.common.Status.SUCCESS
                except IndexError:
                    break
            return py_trees.common.Status.RUNNING
        else:
            self.current_state = ActorExistsActionState.FAILURE
            return py_trees.common.Status.FAILURE

    def on_process_finished(self, _):
        """
        check result of process

        return:
            py_trees.common.Status
        """
        if self.current_state == ActorExistsActionState.WAITING_FOR_ACTOR:
            self.current_state = ActorExistsActionState.FAILURE
            return py_trees.common.Status.FAILURE
        else:
            return py_trees.common.Status.INVALID
