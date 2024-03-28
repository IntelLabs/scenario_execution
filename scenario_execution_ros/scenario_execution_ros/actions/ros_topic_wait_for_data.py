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

import importlib
from scenario_execution_ros.actions.conversions import get_qos_preset_profile, get_clearing_policy
from .py_trees_ros_common import SubscriberWaitForData


class RosTopicWaitForData(SubscriberWaitForData):
    """
    Class to check if the message on ROS topic equals to the target message

    Args:
        topic_name[str]: name of the topic to connect to
        topic_type[str]: class of the message type (e.g. std_msgs.msg.String)
        qos_profile[str]: qos profile for the subscriber
        clearing_policy[str]: when to clear the data
    """

    def __init__(self, name: str, topic_name: str, topic_type: str, qos_profile: str, clearing_policy: str):
        datatype_in_list = topic_type.split(".")
        topic_type = getattr(
            importlib.import_module(".".join(datatype_in_list[0:-1])),
            datatype_in_list[-1]
        )

        super().__init__(
            name=name,
            topic_name=topic_name,
            topic_type=topic_type,
            qos_profile=get_qos_preset_profile(qos_profile),
            clearing_policy=get_clearing_policy(clearing_policy))
