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


from geometry_msgs.msg import PoseWithCovarianceStamped
from lifecycle_msgs.srv import GetState
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.time import Time
from rclpy.duration import Duration
from tf2_ros import Buffer
from datetime import datetime, timedelta
import py_trees

from .nav2_common import NamespaceAwareBasicNavigator
from scenario_execution_ros.actions.common import get_pose_stamped, NamespacedTransformListener
from scenario_execution.actions.base_action import BaseAction, ActionError


class InitNav2State(Enum):
    """
    States for executing a initialization of nav2
    """
    IDLE = 1
    LOCALIZER_STATE_REQUESTED = 2
    LOCALIZER_STATE_RECEIVED = 3
    LOCALIZER_STATE_ACTIVE = 4
    WAIT_FOR_INITIAL_POSE = 5
    WAIT_FOR_MAP_BASELINK_TF = 6
    MAP_BASELINK_TF_RECEIVED = 7
    NAVIGATOR_STATE_REQUESTED = 8
    NAVIGATOR_STATE_RECEIVED = 9
    NAVIGATOR_ACTIVE = 10
    DONE = 11
    FAILURE = 12


class InitNav2(BaseAction):
    """
    Class for initializing nav2 by setting an initial pose and activate required nodes

    """

    def __init__(self, associated_actor, namespace_override: str):
        super().__init__()
        self.initial_pose = None
        self.base_frame_id = None
        self.wait_for_initial_pose = None
        self.use_initial_pose = None
        self.namespace = associated_actor["namespace"]
        self.node = None
        self.future = None
        self.current_state = InitNav2State.IDLE
        self.nav = None
        self.bt_navigator_state_client = None
        self.amcl_state_client = None
        self.localization_pose_sub = None
        self.retry_count = None
        self.service_called_timestamp = None
        self.localizer_state = None
        self.navigator_state = None
        self.tf_buffer = None
        self.tf_listener = None
        self.wait_for_amcl = None
        if namespace_override:
            self.namespace = namespace_override

    def setup(self, **kwargs):
        """
        Setup ROS2 node and service client

        """
        try:
            self.node: Node = kwargs['node']
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(
                self.name, self.__class__.__name__)
            raise ActionError(error_message, action=self) from e

        self.tf_buffer = Buffer()
        self.tf_listener = NamespacedTransformListener(
            node=self.node, buffer=self.tf_buffer, tf_topic=self.namespace + "/tf", tf_static_topic=self.namespace + "/tf_static")

        self.nav = NamespaceAwareBasicNavigator(
            node_name="basic_nav_init_nav2", namespace=self.namespace)
        self.bt_navigator_state_client = self.node.create_client(
            GetState, self.namespace + '/bt_navigator/get_state',
            callback_group=ReentrantCallbackGroup())
        self.amcl_state_client = self.node.create_client(
            GetState, self.namespace + '/amcl/get_state',
            callback_group=ReentrantCallbackGroup())

        amcl_pose_qos = QoSProfile(
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1)

        self.localization_pose_sub = self.node.create_subscription(PoseWithCovarianceStamped,
                                                                   self.namespace + '/amcl_pose',
                                                                   self._amcl_pose_callback,
                                                                   amcl_pose_qos,
                                                                   callback_group=ReentrantCallbackGroup())

    def execute(self, associated_actor, initial_pose: list, base_frame_id: str, wait_for_initial_pose: bool, use_initial_pose: bool, wait_for_amcl: bool):
        self.initial_pose = initial_pose
        self.base_frame_id = base_frame_id
        self.wait_for_initial_pose = wait_for_initial_pose
        self.wait_for_amcl = wait_for_amcl
        self.use_initial_pose = use_initial_pose
        self.namespace = associated_actor["namespace"]

    def update(self) -> py_trees.common.Status:
        """
        Execute states
        """
        self.logger.debug(f"Current State {self.current_state}")
        result = py_trees.common.Status.FAILURE
        if self.current_state == InitNav2State.IDLE:
            if self.wait_for_amcl:
                self.current_state = InitNav2State.LOCALIZER_STATE_REQUESTED
                if self.retry_count is None:
                    self.retry_count = 1000
                else:
                    self.retry_count -= 1
                if self.future:
                    self.amcl_state_client.remove_pending_request(self.future)
                    self.future.cancel()

                if self.retry_count > 0:
                    req = GetState.Request()
                    self.feedback_message = f"Waiting for localizer to become active. Try {1001 - self.retry_count}"  # pylint: disable= attribute-defined-outside-init
                    self.future = self.amcl_state_client.call_async(req)
                    self.service_called_timestamp = datetime.now()
                    self.future.add_done_callback(self._get_state_done_callback)
                    result = py_trees.common.Status.RUNNING
            else:
                self.current_state = InitNav2State.LOCALIZER_STATE_ACTIVE
                result = py_trees.common.Status.RUNNING
        elif self.current_state == InitNav2State.LOCALIZER_STATE_REQUESTED:
            timeout = timedelta(seconds=1)
            if timeout < datetime.now() - self.service_called_timestamp:
                self.feedback_message = f"Localizer state request timed out after 1s. Requesting again..."  # pylint: disable= attribute-defined-outside-init
                self.current_state = InitNav2State.IDLE
            result = py_trees.common.Status.RUNNING
        elif self.current_state == InitNav2State.LOCALIZER_STATE_RECEIVED:
            if self.localizer_state == "active":
                self.current_state = InitNav2State.LOCALIZER_STATE_ACTIVE
                self.retry_count = None
            else:
                self.feedback_message = f"Localizer expected to be active, but is '{self.localizer_state}'. Retrying..."  # pylint: disable= attribute-defined-outside-init
                self.current_state = InitNav2State.IDLE
            result = py_trees.common.Status.RUNNING
        elif self.current_state == InitNav2State.LOCALIZER_STATE_ACTIVE:
            self.current_state = InitNav2State.WAIT_FOR_MAP_BASELINK_TF
            if self.wait_for_initial_pose:
                self.feedback_message = f"Waiting for externally set initial pose."  # pylint: disable= attribute-defined-outside-init
                self.current_state = InitNav2State.WAIT_FOR_INITIAL_POSE
            elif self.use_initial_pose:
                initial_pose = get_pose_stamped(
                    self.nav.get_clock().now().to_msg(), self.initial_pose)
                self.feedback_message = f"Set initial pose."  # pylint: disable= attribute-defined-outside-init
                self.nav.setInitialPose(initial_pose)

                if self.wait_for_amcl:
                    self.current_state = InitNav2State.WAIT_FOR_INITIAL_POSE
            result = py_trees.common.Status.RUNNING
        elif self.current_state == InitNav2State.WAIT_FOR_INITIAL_POSE:
            result = py_trees.common.Status.RUNNING
        elif self.current_state == InitNav2State.WAIT_FOR_MAP_BASELINK_TF:
            if self.tf_buffer.can_transform("map", self.base_frame_id, Time(seconds=0), Duration(seconds=0)):
                self.feedback_message = f"Transform map -> {self.base_frame_id} got available."  # pylint: disable= attribute-defined-outside-init
                self.current_state = InitNav2State.MAP_BASELINK_TF_RECEIVED
            else:
                self.feedback_message = f"Waiting for transform map -> {self.base_frame_id} to get available..."  # pylint: disable= attribute-defined-outside-init
            result = py_trees.common.Status.RUNNING
        elif self.current_state == InitNav2State.MAP_BASELINK_TF_RECEIVED:
            self.current_state = InitNav2State.NAVIGATOR_STATE_REQUESTED
            if self.retry_count is None:
                self.retry_count = 1000
            else:
                self.retry_count -= 1
            if self.future:
                self.bt_navigator_state_client.remove_pending_request(self.future)
                self.future.cancel()
            if self.retry_count > 0:
                req = GetState.Request()
                self.feedback_message = f"Request navigator state. Try {1001 - self.retry_count}"  # pylint: disable= attribute-defined-outside-init
                self.future = self.bt_navigator_state_client.call_async(req)
                self.service_called_timestamp = datetime.now()
                self.future.add_done_callback(self._get_state_done_callback)
                result = py_trees.common.Status.RUNNING
        elif self.current_state == InitNav2State.NAVIGATOR_STATE_REQUESTED:
            timeout = timedelta(seconds=1)
            if timeout < datetime.now() - self.service_called_timestamp:
                self.feedback_message = f"Navigator state request timed out after 1s. Requesting again..."  # pylint: disable= attribute-defined-outside-init
                self.current_state = InitNav2State.MAP_BASELINK_TF_RECEIVED
            result = py_trees.common.Status.RUNNING
        elif self.current_state == InitNav2State.NAVIGATOR_STATE_RECEIVED:
            if self.navigator_state == "active":
                self.feedback_message = f"Navigator active."  # pylint: disable= attribute-defined-outside-init
                self.current_state = InitNav2State.NAVIGATOR_ACTIVE
                self.retry_count = None
            else:
                self.feedback_message = f"Navigator expected to be active, but is {self.navigator_state}. Retrying..."  # pylint: disable= attribute-defined-outside-init
                self.current_state = InitNav2State.MAP_BASELINK_TF_RECEIVED
            result = py_trees.common.Status.RUNNING
        elif self.current_state == InitNav2State.NAVIGATOR_ACTIVE:
            result = py_trees.common.Status.SUCCESS
            self.current_state = InitNav2State.DONE
        elif self.current_state == InitNav2State.DONE:
            self.logger.debug("Nothing to do!")
        else:
            self.logger.error(f"Invalid state {self.current_state}")

        return result

    def _get_state_done_callback(self, future):
        """
        Callback function when the GetState future is done
        """
        self.logger.debug(f"Received state {future.result()}")
        if not future.result():
            return
        if self.current_state == InitNav2State.LOCALIZER_STATE_REQUESTED:
            self.localizer_state = future.result().current_state.label
            self.current_state = InitNav2State.LOCALIZER_STATE_RECEIVED
        elif self.current_state == InitNav2State.NAVIGATOR_STATE_REQUESTED:
            self.navigator_state = future.result().current_state.label
            self.current_state = InitNav2State.NAVIGATOR_STATE_RECEIVED

    def _amcl_pose_callback(self, msg):
        """
        Callback function when amcl pose is received
        """
        if self.current_state == InitNav2State.WAIT_FOR_INITIAL_POSE:
            self.current_state = InitNav2State.WAIT_FOR_MAP_BASELINK_TF
