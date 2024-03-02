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

""" Class for spawn an entity in Gazebo """

import subprocess  # nosec B404
import numpy as np
from enum import Enum
import defusedxml.ElementTree as ET

from transforms3d.taitbryan import euler2quat
from std_msgs.msg import String

from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.logging import get_logger
from rclpy.node import Node
import py_trees
from scenario_execution_base.actions import RunExternalProcess
from scenario_execution_gazebo.utils import SpawnUtils


class SpawnActionState(Enum):
    """
    States for executing a spawn-entity in gazebo
    """
    WAITING_FOR_TOPIC = 1
    MODEL_AVAILABLE = 2
    WAITING_FOR_RESPONSE = 3
    DONE = 4
    FAILURE = 5


class GazeboSpawnMovingActor(RunExternalProcess):

    """Class to spawn a moving entity into simulation.

    Args:
        entity_name [str]: name of spawned entity
        model_file [str]: model file name
        spawn_pose: a 6 numbers list in str form containing the spawn pose of the entity
            in the shape of [x, y, z. roll, pitch, yaw].
            Please pay attention to the Tait-Bryan euler angles ranges.
        actor_path: TODO
    """

    def __init__(self, name, entity_name: str, model_file: str, spawn_pose: list,
                 namespace: str, world_name: str,
                 xacro_arguments: list, trajectory: list):
        """
        init
        """
        super().__init__(name)
        self.entity_name = entity_name
        self.model_file = model_file
        self.spawn_pose = spawn_pose
        self.trajectory_times, self.trajectory_poses = self.extract_trajectory_from_str(trajectory)
        self.namespace = namespace
        self.world_name = world_name
        self.xacro_arguments = xacro_arguments
        self.current_state = SpawnActionState.WAITING_FOR_TOPIC
        self.node = None
        self.logger = None
        self.model_sub = None
        self.sdf = None
        self.utils = None

    def extract_trajectory_from_str(self, trajectory: str):
        s = trajectory.split(',')
        times = [element.split(' ')[0] for element in s]
        poses = [element.split(' ')[1:] for element in s]

        # we expect the trajectory to come as a sequence of the form:
        # time,x,y,z,yaw, so if the lentgh of the poses list is not divisible
        # by 4, there is an error on how the trajectory was specified
        lens = [len(pose) == 4 for pose in poses]
        if np.all(lens):
            return times, poses
        else:
            raise ValueError('The trajectory of %s is not correclty specified' % self.entity_name)

    @staticmethod
    def make_sdf_waypoint(time, pose) -> str:
        """Create an sdf waypoint tag from a time and a pose

        :time: trajectory time
        :pose: trajectory pose
        :returns: and sdf waypoint tag as a string

        """
        waypoint_str = '<waypoint><time>' + time + '</time>'
        waypoint_str += '<pose>' + pose[0] + ' ' + pose[1] + ' ' + pose[2]
        waypoint_str += ' 0 0 ' + pose[3] + '</pose></waypoint>'
        return waypoint_str

    def create_animation_sdf(self):
        """Create an sdf string for an animation

        :returns: sdf string

        """
        animation_str = '<script>'
        # if loop is set to false, Gazebo ignition freezes, therefore it is not
        # supported yet
        # b_loop = 'false'
        # animation_str += '<loop>' + b_loop + '</loop>'
        # animation_str += '<auto_start>true</auto_start>'
        # animation_str += '<delay_start>1.000000</delay_start>'
        animation_str += '<trajectory id=\\"0\\" type=\\"animation\\">'
        for time, pose in zip(self.trajectory_times, self.trajectory_poses):
            animation_str += self.make_sdf_waypoint(time, pose)

        animation_str += '</trajectory></script>'
        return animation_str

    def setup(self, **kwargs):
        """
        Setup ROS2 node and model

        """
        try:
            self.node: Node = kwargs['node']
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(
                self.name, self.__class__.__name__)
            raise KeyError(error_message) from e  # 'direct cause' traceability

        self.logger = get_logger(self.name)
        self.utils = SpawnUtils(logger=self.logger)

        if self.model_file.startswith('topic://'):
            transient_local_qos = QoSProfile(
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                reliability=QoSReliabilityPolicy.RELIABLE,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1)
            topic = self.model_file.replace('topic://', '', 1)
            self.current_state = SpawnActionState.WAITING_FOR_TOPIC
            self.model_sub = self.node.create_subscription(
                String, topic, self.topic_callback, transient_local_qos)
        else:
            sdf = self.utils.parse_model_file(
                self.model_file, self.entity_name, self.xacro_arguments)

            to_parse_sdf = sdf.replace('\\"', '"')
            root = ET.fromstring(to_parse_sdf)
            # script = root.findall('script')

            actor = root.find("./actor")
            if not actor:
                raise ValueError(f"Model {self.model_file} should contain an <actor> element.")

            script = actor.find('./script')
            if script:
                self.logger.info("Found script in actor, removing it...")
                actor.remove(script)

            generated_script_xml = self.create_animation_sdf().replace('\\"', '"')
            generated_script = ET.fromstring(generated_script_xml)
            actor.append(generated_script)

            sdf = ET.tostring(root).decode().replace("\"", "\\\"").replace("\n", "")
            if sdf:
                self.set_command(sdf)
                self.current_state = SpawnActionState.MODEL_AVAILABLE
            else:
                raise ValueError(f'Invalid model specified ({self.model_file})')

    def on_executed(self):
        """
        Hook when process gets executed
        """
        self.current_state = SpawnActionState.WAITING_FOR_RESPONSE
        self.feedback_message = f"Executed spawning, waiting for response..."  # pylint: disable= attribute-defined-outside-init

    def cleanup(self):
        """
        Cleanup on shutdown
        """
        self.logger.info(f"Deleting entity '{self.entity_name}' from simulation.")
        if self.current_state in [SpawnActionState.WAITING_FOR_TOPIC, SpawnActionState.MODEL_AVAILABLE]:
            return

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

    def set_command(self, command):
        """
        Set execution command
        """
        # euler2quat() requires "zyx" convention,
        # while in YAML, we define as pitch-roll-yaw (xyz), since it's more intuitive.
        quaternion = euler2quat(self.spawn_pose[5],
                                self.spawn_pose[4],
                                self.spawn_pose[3])
        pose = '{ position: {' \
            f' x: {self.spawn_pose[0]} y: {self.spawn_pose[1]} z: {self.spawn_pose[2]}' \
            ' } orientation: {' \
            f' w: {quaternion[0]} x: {quaternion[1]} y: {quaternion[2]} z: {quaternion[3]}' \
            ' } }'

        super().set_command(["ign", "service", "-s", "/world/" + self.world_name + "/create",
                             "--reqtype", "ignition.msgs.EntityFactory",
                             "--reptype", "ignition.msgs.Boolean",
                             "--timeout", "30000", "--req", "pose: " + pose + " name: \"" + self.entity_name + "\" allow_renaming: false sdf: \"" + command + "\""])
        self.logger.info(f'Command: {" ".join(self.get_command())}')

    def topic_callback(self, msg):
        '''
        Callback to receive model description from topic
        '''

        self.feedback_message = f"Model received from topic."  # pylint: disable= attribute-defined-outside-init
        self.logger.info("Received robot_description.")
        self.node.destroy_subscription(self.model_sub)
        self.set_command(msg.data.replace("\"", "\\\"").replace("\n", ""))
