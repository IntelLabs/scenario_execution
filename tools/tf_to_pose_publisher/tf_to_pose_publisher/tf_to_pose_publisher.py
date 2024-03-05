#!/usr/bin/env python3
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
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rcl_interfaces.msg import ParameterDescriptor

from tf2_ros import TransformException  # pylint: disable= no-name-in-module
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TransformStamped


class TfToPose(Node):

    def __init__(self):
        super().__init__('tf_to_pose')

        self.parent_frame_id = None
        self.child_frame_id = None
        self.mode = None
        self.last_transform = TransformStamped()

        self.params()

        timer_cb_group = MutuallyExclusiveCallbackGroup()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        timer_period = 0.1
        self._timer = self.create_timer(timer_period, self.timer_callback, callback_group=timer_cb_group)

        self.pose_pub = self.create_publisher(PoseStamped, 'tf_as_pose', 10)

    def params(self):
        """handle ROS parameters and store them as class variables
        :returns: -

        """
        self.declare_parameter('parent_frame_id', 'map', descriptor=ParameterDescriptor(dynamic_typing=True))
        self.parent_frame_id = self.get_parameter_or('parent_frame_id').get_parameter_value().string_value

        self.declare_parameter('child_frame_id', 'base_link', descriptor=ParameterDescriptor(dynamic_typing=True))
        self.child_frame_id = self.get_parameter_or('child_frame_id').get_parameter_value().string_value

        self.declare_parameter('mode', 'continuous', descriptor=ParameterDescriptor(
            dynamic_typing=True))  # choose between continuous and onchange
        self.mode = self.get_parameter_or('mode').get_parameter_value().string_value

    def timer_callback(self):
        """timer callback funtion
        :returns: -

        """
        try:
            t = self.tf_buffer.lookup_transform(
                self.parent_frame_id,
                self.child_frame_id,
                rclpy.time.Time()
            )
        except TransformException as ex:
            self.get_logger().debug(
                f'Could not transform {self.parent_frame_id} to {self.child_frame_id}: {ex}')
            return

        pose = PoseStamped()
        pose.header = t.header
        pose.header.frame_id = self.parent_frame_id

        pose.pose.position.x = t.transform.translation.x
        pose.pose.position.y = t.transform.translation.y
        pose.pose.orientation.x = t.transform.rotation.x
        pose.pose.orientation.y = t.transform.rotation.y
        pose.pose.orientation.z = t.transform.rotation.z
        pose.pose.orientation.w = t.transform.rotation.w

        if self.mode == 'continuous':
            # continuously publish the transform we looked up
            self.pose_pub.publish(pose)

        elif self.mode == 'onchange':
            self.get_logger().debug('tf to pose node is in mode onchanged')
            changed = False
            if t.transform != self.last_transform.transform:
                changed = True

            if changed:
                self.get_logger().debug('transform has changed, so we publish')
                self.last_transform = t
                # only publish transform if it changed
                self.pose_pub.publish(pose)
        else:
            raise ValueError(f"Wrong value for mode: {self.mode}! Possible options are 'continuous' or 'onchange'")


def main(args=None):
    rclpy.init(args=args)
    tf_to_pose_node = TfToPose()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(tf_to_pose_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        tf_to_pose_node.get_logger().info("User requested shut down.")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
