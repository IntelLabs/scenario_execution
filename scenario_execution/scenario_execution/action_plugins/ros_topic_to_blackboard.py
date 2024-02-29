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

""" Scenario execution plugin to put ros data into the blackboard """
from ast import literal_eval
import importlib
import py_trees_ros  # pylint: disable=import-error
from scenario_execution.action_plugins.conversions import get_qos_preset_profile, get_clearing_policy


class RosTopicToBlackboard(py_trees_ros.subscribers.ToBlackboard):
    """
    Class to receive a ros message and save it in the blacbboard

    Args:
        topic_name: name of the topic to connect to
        topic_type: class of the message type (e.g. :obj:`std_msgs.msg.String`)
        qos_profile: qos profile for the subscriber
        blackboard_variables: blackboard variable string or dict {names (keys) -
            message subfields (values)}, use a value of None to indicate the entire message
        initialise_variables: initialise the blackboard variables to some defaults
        clearing_policy: when to clear the data
    """

    def __init__(self,
                 name: str,
                 topic_name: str,
                 topic_type: str,
                 qos_profile: str,
                 blackboard_variables: str,
                 initialise_variables: str,
                 clearing_policy: str
                 ):
        datatype_in_list = topic_type.split(".")
        topic_type = getattr(importlib.import_module(".".join(datatype_in_list[0:-1])), datatype_in_list[-1])
        try:
            if blackboard_variables.startswith('{'):
                blackboard_variables = literal_eval(blackboard_variables.encode('utf-8').decode('unicode_escape'))
        except Exception as e:
            raise ValueError(f"Error while parsing blackboard variables '{blackboard_variables}'") from e

        try:
            initialise_variables = literal_eval(initialise_variables.encode('utf-8').decode('unicode_escape'))
        except Exception as e:
            raise ValueError(f"Error while parsing initialise variables '{initialise_variables}'") from e

        super().__init__(
            name=name,
            topic_name=topic_name,
            topic_type=topic_type,
            qos_profile=get_qos_preset_profile(qos_profile),
            blackboard_variables=blackboard_variables,
            initialise_variables=initialise_variables,
            clearing_policy=get_clearing_policy(clearing_policy))
