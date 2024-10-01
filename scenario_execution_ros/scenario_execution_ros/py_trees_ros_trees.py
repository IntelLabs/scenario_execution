# Software License Agreement (BSD License)
#
# Copyright (c) 2015 Daniel Stonier
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#    * Redistributions of source code must retain the above copyright
#        notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above
#        copyright notice, this list of conditions and the following
#        disclaimer in the documentation and/or other materials provided
#        with the distribution.
#    * Neither the name of Yujin Robot nor the names of its
#        contributors may be used to endorse or promote products derived
#        from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import typing

import py_trees
import py_trees_ros_interfaces.srv as py_trees_srvs  # noqa
import rcl_interfaces.msg as rcl_interfaces_msgs
import rclpy

from py_trees_ros import blackboard
from py_trees_ros import conversions
from py_trees_ros import exceptions
from py_trees_ros import utilities
from py_trees_ros import visitors
from py_trees_ros.trees import BehaviourTree as BehaviourTreeRos
from py_trees_ros_trees import SnapshotStream


class BehaviourTree(BehaviourTreeRos):
    """
    Extend the :class:`py_trees_ros.trees.BehaviourTree` class with a required feature that is not yet available
    as a release.
    
    For details, see:
    https://github.com/splintered-reality/py_trees_ros/commit/247cf47d509f827bb24464be376e8ed2ddefc0fb
    """

    def setup(
            self,
            node: typing.Optional[rclpy.node.Node]=None,
            node_name: str="tree",
            timeout: float=py_trees.common.Duration.INFINITE,
            visitor: typing.Optional[py_trees.visitors.VisitorBase]=None,
            **kwargs: int
    ):
        """
        Copy of py_trees_ros.trees.BehaviourTree plus kwargs forwarding.
        """
        # node creation - can raise rclpy.exceptions.NotInitializedException
        if node:
            # Use existing node if one is passed in, and is of the correct type.
            if isinstance(node, rclpy.node.Node):
                self.node = node
            else:
                raise TypeError(f"invalid node object [received: {type(node)}][expected: rclpy.node.Node]")
        else:
            # Node creation - can raise rclpy.exceptions.NotInitializedException
            self.node = rclpy.create_node(node_name=node_name)
        if visitor is None:
            visitor = visitors.SetupLogger(node=self.node)
        self.default_snapshot_stream_topic_name = SnapshotStream.expand_topic_name(
            node=self.node,
            topic_name="~/snapshots"
        )

        ########################################
        # ROS Comms
        ########################################
        self.snapshot_stream_services = utilities.Services(
            node=self.node,
            service_details=[
                ("close", "~/snapshot_streams/close", py_trees_srvs.CloseSnapshotStream, self._close_snapshot_stream),
                ("open", "~/snapshot_streams/open", py_trees_srvs.OpenSnapshotStream, self._open_snapshot_stream),
                ("reconfigure", "~/snapshot_streams/reconfigure", py_trees_srvs.ReconfigureSnapshotStream, self._reconfigure_snapshot_stream),
            ],
            introspection_topic_name="snapshot_streams/services"
        )
        self.blackboard_exchange = blackboard.Exchange()
        self.blackboard_exchange.setup(self.node)

        ################################################################################
        # Parameters
        ################################################################################
        self.node.add_on_set_parameters_callback(
            callback=self._set_parameters_callback
        )

        ########################################
        # default_snapshot_stream
        ########################################
        self.node.declare_parameter(
            name='default_snapshot_stream',
            value=False,
            descriptor=rcl_interfaces_msgs.ParameterDescriptor(
                name="default_snapshot_stream",
                type=rcl_interfaces_msgs.ParameterType.PARAMETER_BOOL,  # noqa
                description="enable/disable the default snapshot stream in ~/snapshots",
                additional_constraints="",
                read_only=False,
            )
        )

        ########################################
        # default_snapshot_period
        ########################################
        self.node.declare_parameter(
            name='default_snapshot_period',
            value=2.0,  # DJS: py_trees.common.Duration.INFINITE.value,
            descriptor=rcl_interfaces_msgs.ParameterDescriptor(
                name="default_snapshot_period",
                type=rcl_interfaces_msgs.ParameterType.PARAMETER_DOUBLE,  # noqa
                description="time between snapshots, set to math.inf to only publish tree state changes",
                additional_constraints="",
                read_only=False,
                floating_point_range=[rcl_interfaces_msgs.FloatingPointRange(
                    from_value=0.0,
                    to_value=py_trees.common.Duration.INFINITE.value)]
            )
        )

        ########################################
        # default_snapshot_blackboard_data
        ########################################
        self.node.declare_parameter(
            name='default_snapshot_blackboard_data',
            value=True,
            descriptor=rcl_interfaces_msgs.ParameterDescriptor(
                name="default_snapshot_blackboard_data",
                type=rcl_interfaces_msgs.ParameterType.PARAMETER_BOOL,  # noqa
                description="append blackboard data (tracking status, visited variables) to the default snapshot stream",
                additional_constraints="",
                read_only=False,
            )
        )

        ########################################
        # default_snapshot_blackboard_activity
        ########################################
        self.node.declare_parameter(
            name='default_snapshot_blackboard_activity',
            value=False,
            descriptor=rcl_interfaces_msgs.ParameterDescriptor(
                name="default_snapshot_blackboard_activity",
                type=rcl_interfaces_msgs.ParameterType.PARAMETER_BOOL,  # noqa
                description="append the blackboard activity stream to the default snapshot stream",
                additional_constraints="",
                read_only=False,
            )
        )

        ########################################
        # setup_timeout
        ########################################
        self.node.declare_parameter(
            name='setup_timeout',
            value=timeout if timeout != py_trees.common.Duration.INFINITE else py_trees.common.Duration.INFINITE.value,
            descriptor=rcl_interfaces_msgs.ParameterDescriptor(
                name="setup_timeout",
                type=rcl_interfaces_msgs.ParameterType.PARAMETER_DOUBLE,  # noqa
                description="timeout for ROS tree setup (node, pubs, subs, ...)",
                additional_constraints="",
                read_only=True,
                floating_point_range=[rcl_interfaces_msgs.FloatingPointRange(
                    from_value=0.0,
                    to_value=py_trees.common.Duration.INFINITE.value)]
            )
        )
        # Get the resulting timeout
        setup_timeout = self.node.get_parameter("setup_timeout").value
        # Ugly workaround to accomodate use of the enum (TODO: rewind this)
        #   Need to pass the enum for now (instead of just a float) in case
        #   there are behaviours out in the wild that apply logic around the
        #   use of the enum
        if setup_timeout == py_trees.common.Duration.INFINITE.value:
            setup_timeout = py_trees.common.Duration.INFINITE

        ########################################
        # Behaviours
        ########################################
        try:
            super().setup(
                timeout=setup_timeout,
                visitor=visitor,
                node=self.node,
                **kwargs
            )
        except RuntimeError as e:
            if str(e) == "tree setup interrupted or timed out":
                raise exceptions.TimedOutError(str(e))
            else:
                raise

        ########################################
        # Setup Handlers
        ########################################
        # set a handler to publish future modifications whenever the tree is modified
        # (e.g. pruned). The tree_update_handler method is in the base class, set this
        # to the callback function here.
        self.tree_update_handler = self._on_tree_update_handler
        self.post_tick_handlers.append(self._snapshots_post_tick_handler)
