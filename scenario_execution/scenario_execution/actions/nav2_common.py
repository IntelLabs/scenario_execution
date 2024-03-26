# Copyright 2021 Samsung Research America
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

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import SingleThreadedExecutor

from transforms3d.taitbryan import euler2quat

from nav2_msgs.action import BackUp, Spin
from nav2_msgs.action import ComputePathThroughPoses, ComputePathToPose
from nav2_msgs.action import FollowPath, FollowWaypoints, NavigateThroughPoses, NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap, GetCostmap, LoadMap

from nav2_simple_commander.robot_navigator import BasicNavigator  # pylint: disable=import-error
from tf2_ros import Buffer
from tf2_msgs.msg import TFMessage
from threading import Thread

from typing import Optional
from typing import Union


def get_pose_stamped(timestamp, in_pose):
    """
    Convert pose list to PoseStamped
    """
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = timestamp
    pose.pose.position.x = float(in_pose['position']['x'])
    pose.pose.position.y = float(in_pose['position']['y'])
    pose.pose.position.z = float(in_pose['position']['z'])

    # euler2quat() requires "zyx" convention,
    # while in YAML, we define as pitch-roll-yaw (xyz), since it's more intuitive.
    quaternion = euler2quat(float(in_pose['orientation']['yaw']),
                            float(in_pose['orientation']['roll']),
                            float(in_pose['orientation']['pitch']))
    pose.pose.orientation.w = quaternion[0]
    pose.pose.orientation.x = quaternion[1]
    pose.pose.orientation.y = quaternion[2]
    pose.pose.orientation.z = quaternion[3]
    return pose


class NamespacedTransformListener:
    """
    :class:`TransformListener` is a convenient way to listen for coordinate frame transformation info.
    This class takes an object that instantiates the :class:`BufferInterface` interface, to which
    it propagates changes to the tf frame graph.
    """

    def __init__(
        self,
        buffer: Buffer,
        node: Node,
        *,
        tf_topic: str = '/tf',
        tf_static_topic: str = '/tf_static',
        spin_thread: bool = False,
        qos: Optional[Union[QoSProfile, int]] = None,
        static_qos: Optional[Union[QoSProfile, int]] = None
    ) -> None:
        """
        Constructor.

        :param buffer: The buffer to propagate changes to when tf info updates.
        :param node: The ROS2 node.
        :param spin_thread: Whether to create a dedidcated thread to spin this node.
        :param qos: A QoSProfile or a history depth to apply to subscribers.
        :param static_qos: A QoSProfile or a history depth to apply to tf_static subscribers.
        """
        if qos is None:
            qos = QoSProfile(
                depth=100,
                durability=DurabilityPolicy.VOLATILE,
                history=HistoryPolicy.KEEP_LAST,
            )
        if static_qos is None:
            static_qos = QoSProfile(
                depth=100,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                history=HistoryPolicy.KEEP_LAST,
            )
        self.buffer = buffer
        self.node = node
        # Default callback group is mutually exclusive, which would prevent waiting for transforms
        # from another callback in the same group.
        self.group = ReentrantCallbackGroup()
        self.tf_sub = node.create_subscription(
            TFMessage, tf_topic, self.callback, qos, callback_group=self.group)
        self.tf_static_sub = node.create_subscription(
            TFMessage, tf_static_topic, self.static_callback, static_qos, callback_group=self.group)

        if spin_thread:
            self.executor = SingleThreadedExecutor()

            def run_func():
                self.executor.add_node(self.node)
                self.executor.spin()
                self.executor.remove_node(self.node)

            self.dedicated_listener_thread = Thread(target=run_func)
            self.dedicated_listener_thread.start()

    def __del__(self) -> None:
        if hasattr(self, 'dedicated_listener_thread') and hasattr(self, 'executor'):
            self.executor.shutdown()
            self.dedicated_listener_thread.join()

        self.unregister()

    def unregister(self) -> None:
        """
        Unregisters all tf subscribers.
        """
        try:
            self.node.destroy_subscription(self.tf_sub)
            self.node.destroy_subscription(self.tf_static_sub)
        except AttributeError:
            pass

    def callback(self, data: TFMessage) -> None:
        who = 'default_authority'
        for transform in data.transforms:
            self.buffer.set_transform(transform, who)

    def static_callback(self, data: TFMessage) -> None:
        who = 'default_authority'
        for transform in data.transforms:
            self.buffer.set_transform_static(transform, who)


class NamespaceAwareBasicNavigator(BasicNavigator):
    """
    Subclass of BasicNavigator to support namespaces
    """

    def __init__(self, node_name, namespace):
        """
        Clone of BasicNavigator init + support for namespaces
        """
        super(BasicNavigator, self).__init__(node_name=node_name,  # pylint: disable=bad-super-call
                                             namespace=namespace)  # pylint: disable=non-parent-init-called

        self.initial_pose = PoseStamped()
        self.initial_pose.header.frame_id = 'map'
        self.goal_handle = None
        self.result_future = None
        self.feedback = None
        self.status = None
        self.cb_group = ReentrantCallbackGroup()

        amcl_pose_qos = QoSProfile(
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1)

        self.initial_pose_received = False
        self.nav_through_poses_client = ActionClient(self,
                                                     NavigateThroughPoses,
                                                     'navigate_through_poses')
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.follow_waypoints_client = ActionClient(self, FollowWaypoints, 'follow_waypoints')
        self.follow_path_client = ActionClient(self, FollowPath, 'follow_path')
        self.compute_path_to_pose_client = ActionClient(self, ComputePathToPose,
                                                        'compute_path_to_pose')
        self.compute_path_through_poses_client = ActionClient(self, ComputePathThroughPoses,
                                                              'compute_path_through_poses')
        self.spin_client = ActionClient(self, Spin, 'spin')
        self.backup_client = ActionClient(self, BackUp, 'backup')
        self.localization_pose_sub = self.create_subscription(PoseWithCovarianceStamped,
                                                              'amcl_pose',
                                                              self._amclPoseCallback,
                                                              amcl_pose_qos)
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped,
                                                      'initialpose',
                                                      10)
        self.change_maps_srv = self.create_client(LoadMap, 'map_server/load_map')
        self.clear_costmap_global_srv = self.create_client(
            ClearEntireCostmap, 'global_costmap/clear_entirely_global_costmap',
            callback_group=self.cb_group)
        self.clear_costmap_local_srv = self.create_client(
            ClearEntireCostmap, 'local_costmap/clear_entirely_local_costmap',
            callback_group=self.cb_group)
        self.get_costmap_global_srv = self.create_client(
            GetCostmap, 'global_costmap/get_costmap', callback_group=self.cb_group)
        self.get_costmap_local_srv = self.create_client(
            GetCostmap, 'local_costmap/get_costmap', callback_group=self.cb_group)
