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

import rclpy
from math import sin, cos, atan2
from tf2_ros import TransformException  # pylint: disable= no-name-in-module
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_geometry_msgs import PoseStamped
from .gazebo_spawn_actor import GazeboSpawnActor


class GazeboRelativeSpawnActor(GazeboSpawnActor):
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
        super().__init__(name, associated_actor, [], world_name, xacro_arguments, model)

        self.frame_id = frame_id
        self.parent_frame_id = parent_frame_id
        self.distance = distance
        self._pose = '{}'
        self.tf_buffer = Buffer()
        self.tf_listener = None

    def setup(self, **kwargs):
        super().setup(**kwargs)
        self.tf_listener = TransformListener(self.tf_buffer, self.node)

    def get_spawn_pose(self):
        self.calculate_new_pose()
        return self._pose

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
                timeout=rclpy.duration.Duration(seconds=0.0),
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
        except TransformException as e:
            raise ValueError(f"No transform available ({self.parent_frame_id}->{self.frame_id})") from e
