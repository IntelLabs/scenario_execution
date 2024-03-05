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
import py_trees_ros  # pylint: disable=import-error
from scenario_execution.actions.conversions import get_qos_preset_profile, \
    get_comparison_operator, get_clearing_policy


class RosTopicCheckData(py_trees_ros.subscribers.CheckData):
    """
    Class to check if the message on ROS topic equals to the target message
    """

    def __init__(self,
                 name: str,
                 topic_name: str,
                 topic_type: str,
                 qos_profile: str,
                 variable_name: str,
                 expected_value: str,
                 comparison_operator: int,
                 fail_if_no_data: bool,
                 fail_if_bad_comparison: bool,
                 clearing_policy: int
                 ):
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
            variable_name=variable_name,
            expected_value=expected_value,
            comparison_operator=get_comparison_operator(comparison_operator),
            fail_if_no_data=fail_if_no_data,
            fail_if_bad_comparison=fail_if_bad_comparison,
            clearing_policy=get_clearing_policy(clearing_policy))
