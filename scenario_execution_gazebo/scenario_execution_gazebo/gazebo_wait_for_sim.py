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

""" Class to wait for sim to start """

import py_trees
from enum import Enum

from scenario_execution_base.actions import RunExternalProcess


class WaitForSimulationActionState(Enum):
    """
    States for waiting for the simulation
    """
    IDLE = 1
    WAITING_FOR_SIM = 2
    DONE = 3
    FAILURE = 4


class GazeboWaitForSim(RunExternalProcess):
    """
    Class to wait for the simulation to become active

    Args:
        world_name [str]: name of simulation world

    """

    def __init__(self, name, world_name: str, **kwargs):
        self.node = None
        self.world_name = world_name
        command = ["ign", "topic", "-t", "/world/" +
                   world_name + "/clock", "-e", "--json-output", "-n", "1"]
        super().__init__(name, command)
        self.current_state = WaitForSimulationActionState.IDLE

    def on_executed(self):
        """
        Hook when process gets executed
        """
        self.feedback_message = f"Waiting for simulation of world '{self.world_name}'"  # pylint: disable= attribute-defined-outside-init
        self.current_state = WaitForSimulationActionState.WAITING_FOR_SIM

    def get_logger_stdout(self):
        """
        get logger for stderr messages
        """
        return None

    def on_process_finished(self, _):
        """
        check result of process

        return:
            py_trees.common.Status
        """
        if self.current_state == WaitForSimulationActionState.WAITING_FOR_SIM:
            self.feedback_message = f"Simulation is running"  # pylint: disable= attribute-defined-outside-init
            self.current_state = WaitForSimulationActionState.DONE
            return py_trees.common.Status.SUCCESS
        else:
            self.current_state = WaitForSimulationActionState.FAILURE
            return py_trees.common.Status.FAILURE
