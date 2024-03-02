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

import py_trees
import py_trees_ros  # pylint: disable=import-error
import typing
import rclpy.qos


class SubscriberHandler(py_trees_ros.subscribers.Handler):
    """
    overrides Handler
    """

    def __init__(self,
                 name: str,
                 topic_name: str,
                 topic_type: typing.Any,
                 qos_profile: rclpy.qos.QoSProfile,
                 clearing_policy: py_trees.common.ClearingPolicy = py_trees.common.ClearingPolicy.ON_INITIALISE
                 ):
        """
        override
        """
        super(SubscriberHandler, self).__init__(name=name, topic_name=topic_name,
                                                topic_type=topic_type, qos_profile=qos_profile, clearing_policy=clearing_policy)

    def setup(self, **kwargs):
        """
        Initialises the subscriber.
        """
        try:
            self.node = kwargs['node']  # pylint: disable= attribute-defined-outside-init
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(
                self.name, self.__class__.__name__)
            raise KeyError(error_message) from e
        self.subscriber = self.node.create_subscription(  # pylint: disable= attribute-defined-outside-init
            msg_type=self.topic_type,
            topic=self.topic_name,
            callback=self._callback,
            qos_profile=self.qos_profile,
            callback_group=rclpy.callback_groups.ReentrantCallbackGroup()
        )
