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
from enum import Enum

from transforms3d.taitbryan import euler2quat
from std_msgs.msg import String

from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.logging import get_logger
from rclpy.node import Node
import py_trees
from scenario_execution.actions.run_process import RunProcess
from .utils import SpawnUtils


class SpawnActionState(Enum):
    """
    States for executing a spawn-entity in gazebo
    """
    WAITING_FOR_TOPIC = 1
    MODEL_AVAILABLE = 2
    WAITING_FOR_RESPONSE = 3
    DONE = 4
    FAILURE = 5


class GazeboSpawnActor(RunProcess):
    """
    Class to spawn an entity into simulation

    """

    def __init__(self, associated_actor, spawn_pose: list, world_name: str, xacro_arguments: list, model: str, **kwargs):
        """
        init
        """
        super().__init__(name, "")
        self.entity_name = associated_actor["name"]
        self.model = model
        self.spawn_pose = spawn_pose
        self.world_name = world_name
        self.xacro_arguments = xacro_arguments
        self.current_state = SpawnActionState.WAITING_FOR_TOPIC
        self.node = None
        self.logger = None
        self.model_sub = None
        self.sdf = None
        self.utils = None

    def setup(self, **kwargs):
        """
        Setup ROS2 node and model

        """
        try:
            self.node: Node = kwargs['node']
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(
                self.name, self.__class__.__name__)
            raise KeyError(error_message) from e

        self.logger = get_logger(self.name)
        self.utils = SpawnUtils(logger=self.logger)

        if self.model.startswith('topic://'):
            transient_local_qos = QoSProfile(
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                reliability=QoSReliabilityPolicy.RELIABLE,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1)
            topic = self.model.replace('topic://', '', 1)
            self.current_state = SpawnActionState.WAITING_FOR_TOPIC
            self.feedback_message = f"Waiting for model on topic {topic}"  # pylint: disable= attribute-defined-outside-init
            self.model_sub = self.node.create_subscription(
                String, topic, self.topic_callback, transient_local_qos)
        else:
            self.sdf = self.utils.parse_model_file(
                self.model, self.entity_name, self.xacro_arguments)

            if not self.sdf:
                raise ValueError(f'Invalid model specified ({self.model})')
            self.current_state = SpawnActionState.MODEL_AVAILABLE

    def update(self) -> py_trees.common.Status:
        """
        Send request
        """
        if self.current_state == SpawnActionState.WAITING_FOR_TOPIC:
            return py_trees.common.Status.RUNNING
        elif self.current_state == SpawnActionState.MODEL_AVAILABLE:
            try:
                self.set_command(self.sdf)
                self.current_state = SpawnActionState.WAITING_FOR_RESPONSE
                return super().update()
            except ValueError as e:
                self.feedback_message = str(e)  # pylint: disable= attribute-defined-outside-init
                self.current_state = SpawnActionState.FAILURE
                return py_trees.common.Status.FAILURE
        else:
            return super().update()

    def on_executed(self):
        """
        Hook when process gets executed
        """
        self.feedback_message = f"Executed spawning, waiting for response..."  # pylint: disable= attribute-defined-outside-init

    def shutdown(self):
        """
        Cleanup on shutdown
        """
        if self.current_state in [SpawnActionState.WAITING_FOR_TOPIC, SpawnActionState.MODEL_AVAILABLE]:
            return

        self.logger.info(f"Deleting entity '{self.entity_name}' from simulation.")
        subprocess.run(["ign", "service", "-s", "/world/" + self.world_name + "/remove",  # pylint: disable=subprocess-run-check
                        "--reqtype", "ignition.msgs.Entity",
                        "--reptype", "ignition.msgs.Boolean",
                        "--timeout", "1000", "--req", "name: \"" + self.entity_name + "\" type: MODEL"])

    def on_process_finished(self, ret):
        """
        check result of process

        return:
            py_trees.common.Status
        """
        if self.current_state == SpawnActionState.WAITING_FOR_RESPONSE:
            if ret == 0:
                while True:
                    try:
                        line = self.output.popleft()
                        line = line.lower()
                        if 'error' in line or 'timed out' in line:
                            self.logger.warn(line)
                            self.feedback_message = f"Found error output while executing '{self.command}'"  # pylint: disable= attribute-defined-outside-init
                            self.current_state = SpawnActionState.FAILURE
                            return py_trees.common.Status.FAILURE
                    except IndexError:
                        break
                self.current_state = SpawnActionState.DONE
                return py_trees.common.Status.SUCCESS
            else:
                self.current_state = SpawnActionState.FAILURE
                return py_trees.common.Status.FAILURE
        else:
            return py_trees.common.Status.INVALID

    def get_spawn_pose(self):
        # euler2quat() requires "zyx" convention,
        # while in YAML, we define as pitch-roll-yaw (xyz), since it's more intuitive.
        try:
            quaternion = euler2quat(self.spawn_pose["orientation"]["yaw"],
                                    self.spawn_pose["orientation"]["roll"],
                                    self.spawn_pose["orientation"]["pitch"])
            pose = '{ position: {' \
                f' x: {self.spawn_pose["position"]["x"]} y: {self.spawn_pose["position"]["y"]} z: {self.spawn_pose["position"]["z"]}' \
                ' } orientation: {' \
                f' w: {quaternion[0]} x: {quaternion[1]} y: {quaternion[2]} z: {quaternion[3]}' \
                ' } }'
        except KeyError as e:
            raise ValueError("Could not get values") from e
        return pose

    def set_command(self, command):
        """
        Set execution command
        """
        pose = self.get_spawn_pose()

        super().set_command(["ign", "service", "-s", "/world/" + self.world_name + "/create",
                             "--reqtype", "ignition.msgs.EntityFactory",
                             "--reptype", "ignition.msgs.Boolean",
                             "--timeout", "30000", "--req", "pose: " + pose + " name: \"" + self.entity_name + "\" allow_renaming: false sdf: \"" + command + "\""])

    def topic_callback(self, msg):
        '''
        Callback to receive model description from topic
        '''

        self.feedback_message = f"Model received from topic."  # pylint: disable= attribute-defined-outside-init
        self.logger.info("Received robot_description.")
        self.node.destroy_subscription(self.model_sub)
        self.set_command(msg.data.replace("\"", "\\\"").replace("\n", ""))
        self.current_state = SpawnActionState.MODEL_AVAILABLE
