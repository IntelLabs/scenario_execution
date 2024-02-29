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

""" Scenario execution plugin to write data to the blackboard on reception of empty ros message """
import py_trees_ros  # pylint: disable=import-error
from scenario_execution.action_plugins.conversions import get_qos_preset_profile


class RosEventToBlackboard(py_trees_ros.subscribers.EventToBlackboard):
    """
    Class to receive a empty ros message and publish to blackboard

    Args:
        topic_name: name of the topic to connect to
        qos_profile: qos profile for the subscriber
        variable_name: name to write the boolean result on the blackboard
    """

    def __init__(self,
                 name: str,
                 topic_name: str,
                 qos_profile: str,
                 variable_name: str,
                 ):

        super().__init__(
            name=name,
            topic_name=topic_name,
            qos_profile=get_qos_preset_profile(qos_profile),
            variable_name=variable_name)
