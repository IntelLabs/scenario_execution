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

"""node to publish the status of the running scenario to a ROS topic"""
#!/usr/bin/env python3

import rclpy
from rclpy import qos
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor

from py_trees_ros_interfaces.srv import OpenSnapshotStream
from py_trees_ros_interfaces.msg import BehaviourTree
from scenario_execution_interfaces.msg import ScenarioStatus as ScenarioStatusMsg  # pylint: disable=no-name-in-module
from rosgraph_msgs.msg import Clock


class ScenarioStatus(Node):

    """Simple node to call a service to publish the py-trees-behaviour tree
    to a topic, then subscribe to that topic and publish changes in behaviour
    states as strings at the time they are happening."""

    def __init__(self):
        """initialize node"""
        super().__init__('scenario_status')

        self.bt_topic = None
        self.snapshot_srv_name = None
        self.scenario_status_topic = None
        self.behaviour_infos = None
        self.last_behaviour_infos = None
        self.scenario_bt = None
        self.time = None
        # self.client_node = None
        self.logger = self.get_logger()

        self.states_dict = {
            1: 'INVALID',
            2: 'RUNNING',
            3: 'SUCCESS',
            4: 'FAILURE'
        }
        self.types_dict = {
            # Possible types of behaviour
            0: 'UNKNOWN_TYPE',
            1: 'BEHAVIOUR',
            2: 'SEQUENCE',
            3: 'SELECTOR',
            4: 'PARALLEL',
            5: 'CHOOSER',
            6: 'DECORATOR'
        }
        self.params()

        clock_qos = qos.QoSProfile(
            durability=qos.QoSDurabilityPolicy.VOLATILE,
            reliability=qos.QoSReliabilityPolicy.BEST_EFFORT,
            history=qos.QoSHistoryPolicy.KEEP_LAST,
            depth=5)
        self.clock_sub = self.create_subscription(Clock, '/clock', self.clock_callback, clock_qos)
        self.status_pub = self.create_publisher(ScenarioStatusMsg, self.scenario_status_topic, 10)

        # self.request_bt_publishing()

        self.snapshot_sub = self.create_subscription(
            BehaviourTree,
            self.bt_topic,
            self.snapshot_callback,
            10
        )

    def clock_callback(self, msg):
        self.time = msg

    def request_bt_publishing(self):
        """Request publishing of the py-trees-behaviour tree via the given
        topic name vy calling the corresponding service.
        :returns: -

        """
        # self.client_node = rclpy.create_node('srv_client_node')
        snapshot_client = self.create_client(OpenSnapshotStream, self.snapshot_srv_name)
        while not snapshot_client.wait_for_service(timeout_sec=1.0):
            self.logger.info(
                'Service to open bt snapshot not available, waiting again ...')

        req = OpenSnapshotStream.Request(topic_name=self.bt_topic)
        req.topic_name = self.bt_topic
        # calling service to publish pytrees behaviour tree snapshot to topic
        future = snapshot_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.bt_topic = future.result().topic_name
            self.logger.info(
                'Triggered publishing of bt snapshot to topic: %s' % self.bt_topic)
        else:
            self.logger.error('Exception while calling service: {}'.format(future.exception()))

    def params(self):
        """handle ROS parameters and store them as class variables
        :returns: -

        """
        self.declare_parameter('bt_snapshot_topic', '/bt_snapshot',
                               descriptor=ParameterDescriptor(dynamic_typing=True))
        self.declare_parameter('snapshot_srv_name', '/scenario_execution/snapshot_streams/open',
                               descriptor=ParameterDescriptor(dynamic_typing=True))
        self.declare_parameter('scenario_status_topic', '/scenario_status',
                               descriptor=ParameterDescriptor(dynamic_typing=True))

        self.bt_topic = self.get_parameter_or(
            'bt_snapshot_topic').get_parameter_value().string_value
        self.snapshot_srv_name = self.get_parameter_or(
            'snapshot_srv_name').get_parameter_value().string_value
        self.scenario_status_topic = self.get_parameter_or(
            'scenario_status_topic').get_parameter_value().string_value

    @staticmethod
    def get_behaviour_infos(behaviour_tree_msg):
        """get information about behaviour tree as dictionary
        :returns: dictionary containing information

        """
        result = {}
        for behaviour in behaviour_tree_msg.behaviours:
            result[behaviour.name] = {
                'class_name': behaviour.class_name,
                'status': behaviour.status,
                'is_active': behaviour.is_active,
                'type': behaviour.type,
                'message': behaviour.message
            }
        return result

    def snapshot_callback(self, msg):
        """callback for the behaviour tree snapshop topic

        :msg: incoming behaviour tree msg
        :returns: -

        """
        current_time = None
        if self.time:
            current_time = self.time.clock
        self.logger.debug('received bt_snapshot')
        if self.scenario_bt is not None:
            self.last_behaviour_infos = self.get_behaviour_infos(
                self.scenario_bt
            )
        self.scenario_bt = msg
        self.behaviour_infos = self.get_behaviour_infos(self.scenario_bt)
        # we can only check for changes if we have a previous bt available
        if self.last_behaviour_infos is not None:
            for behaviour, infos in self.behaviour_infos.items():
                if infos['status'] not in self.states_dict or self.last_behaviour_infos[behaviour]['status'] not in self.states_dict:
                    continue
                if infos['status'] != self.last_behaviour_infos[behaviour]['status']:
                    beh_type = self.types_dict[infos['type']]
                    last_status = self.states_dict[self.last_behaviour_infos[behaviour]['status']]
                    current_status = self.states_dict[infos['status']]
                    result_str = f'{behaviour}({beh_type}): {last_status} > {current_status}'
                    debug_str = 'behaviour %s of type %s changed state from %s to %s, with message %s' % (
                        behaviour, beh_type, last_status, current_status, infos['message'])
                    msg = ScenarioStatusMsg()
                    msg.data = result_str
                    msg.system_time = self.get_clock().now().to_msg()
                    if current_time:
                        msg.ros_time = current_time
                    self.logger.debug(debug_str)
                    self.status_pub.publish(msg)


def main(args=None):
    """main function for the node to spin
    """
    rclpy.init(args=args)

    scenario_status = ScenarioStatus()

    try:
        scenario_status.request_bt_publishing()
        rclpy.spin(scenario_status)
    except KeyboardInterrupt:
        pass
    finally:
        scenario_status.destroy_node()


if __name__ == "__main__":
    main()
