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

from std_msgs.msg import String

import rclpy
from math import sin, cos, atan2
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.logging import get_logger
from rclpy.node import Node
from tf2_ros import TransformException  # pylint: disable= no-name-in-module
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_geometry_msgs import PoseStamped
import py_trees
from scenario_execution.actions.run_process import RunProcess
from .utils import SpawnUtils


class SpawnActionState(Enum):
    """
    States for executing a spawn-entity in gazebo
    """
    WAITING_FOR_TOPIC = 1
    WAITING_FOR_POSE = 2
    POSE_AVAILABLE = 3
    MODEL_AVAILABLE = 4
    WAITING_FOR_RESPONSE = 5
    DONE = 6
    FAILURE = 7


class GazeboRelativeSpawnActor(RunProcess):
    """
    Class to spawn an entity into simulation

    """

    def __init__(self, name, associated_actor,
                 frame_id: str, parent_frame_id: str,
                 distance: float, world_name: str, xacro_arguments: list,
                 model: str, **kwargs):
        """
        init
        """
        super().__init__(name, "")
        self.entity_name = associated_actor["name"]
        self.model_file = model
        self.frame_id = frame_id
        self.parent_frame_id = parent_frame_id
        self.distance = distance
        self.world_name = world_name
        self.xacro_arguments = xacro_arguments
        self.current_state = SpawnActionState.WAITING_FOR_TOPIC
        self.node = None
        self.logger = None
        self.model_sub = None
        self.sdf = None
        self.utils = None
        self._pose = None
        self.sdf = None
        self.tf_buffer = Buffer()

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
        _ = TransformListener(self.tf_buffer, self.node)

        if self.model_file.startswith('topic://'):
            transient_local_qos = QoSProfile(
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                reliability=QoSReliabilityPolicy.RELIABLE,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1)
            topic = self.model_file.replace('topic://', '', 1)
            self.current_state = SpawnActionState.WAITING_FOR_TOPIC
            self.feedback_message = f"Waiting for model on topic {topic}"  # pylint: disable= attribute-defined-outside-init
            self.model_sub = self.node.create_subscription(
                String, topic, self.topic_callback, transient_local_qos)
        else:
            self.sdf = self.utils.parse_model_file(
                self.model_file, self.entity_name, self.xacro_arguments)
            self.current_state = SpawnActionState.WAITING_FOR_POSE

    def update(self) -> py_trees.common.Status:
        """
        Send request
        """
        if self.current_state == SpawnActionState.WAITING_FOR_TOPIC:
            return py_trees.common.Status.RUNNING
        elif self.current_state == SpawnActionState.WAITING_FOR_POSE:
            self.calculate_new_pose()
            return py_trees.common.Status.RUNNING
        elif self.current_state == SpawnActionState.POSE_AVAILABLE:
            if self.sdf:
                self.set_command(self.sdf)
            else:
                raise ValueError(f'Invalid model specified ({self.model_file})')
            return py_trees.common.Status.RUNNING
        else:
            return super().update()

    def on_executed(self):
        """
        Hook when process gets executed
        """
        self.current_state = SpawnActionState.WAITING_FOR_RESPONSE
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

    def calculate_new_pose(self):
        """
        Get position of the frame with frame_id relative to the parent_frame_id
        and create a pose with specified distance in front of it

        """
        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(
                self.parent_frame_id,
                self.frame_id,
                now,
                timeout=rclpy.duration.Duration(seconds=1.0),
            )

            # Extract current position and orientation
            current_position = trans.transform.translation
            current_orientation = trans.transform.rotation

            rotation_angle = 2 * atan2(current_orientation.z, current_orientation.w)

            # Calculate new position with distance in front
            new_x = current_position.x + self.distance * cos(rotation_angle)
            new_y = current_position.y + self.distance * sin(rotation_angle)

            # Create new pose
            new_pose = PoseStamped()
            new_pose.header.stamp = now.to_msg()
            new_pose.header.frame_id = self.parent_frame_id
            new_pose.pose.position.x = new_x
            new_pose.pose.position.y = new_y
            new_pose.pose.position.z = current_position.z  # Assuming same height

            # The orientation remains the same
            new_pose.pose.orientation = current_orientation

            self._pose = '{ position: {' \
                f' x: {new_pose.pose.position.x} y: {new_pose.pose.position.y} z: {new_pose.pose.position.z}' \
                ' } orientation: {' \
                f' w: {new_pose.pose.orientation.w} x: {new_pose.pose.orientation.x} y: {new_pose.pose.orientation.y} z: {new_pose.pose.orientation.z}' \
                ' } }'

            self.current_state = SpawnActionState.POSE_AVAILABLE

        except TransformException as ex:
            self.feedback_message = f"Could not transform {self.parent_frame_id} to {self.frame_id}"  # pylint: disable= attribute-defined-outside-init
            self.logger().warn(
                f'Could not transform {self.parent_frame_id} to {self.frame_id} at time {now}: {ex}')

    def set_command(self, command):
        """
        Set execution command
        """
        super().set_command(["ign", "service", "-s", "/world/" + self.world_name + "/create",
                             "--reqtype", "ignition.msgs.EntityFactory",
                             "--reptype", "ignition.msgs.Boolean",
                             "--timeout", "30000", "--req", "pose: " + str(self._pose) + " name: \"" + self.entity_name + "\" allow_renaming: true sdf: \"" + command + "\""])

        self.logger.info(f'Command: {" ".join(self.get_command())}')
        self.current_state = SpawnActionState.MODEL_AVAILABLE

    def topic_callback(self, msg):
        '''
        Callback to receive model description from topic
        '''

        self.feedback_message = f"Model received from topic."  # pylint: disable= attribute-defined-outside-init
        self.logger.info("Received robot_description.")
        self.node.destroy_subscription(self.model_sub)
        self.set_command(msg.data.replace("\"", "\\\"").replace("\n", ""))
        self.current_state = SpawnActionState.WAITING_FOR_POSE
