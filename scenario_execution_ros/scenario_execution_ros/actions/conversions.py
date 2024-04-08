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

""" common conversions """

import operator
from rclpy.qos import QoSPresetProfiles
import py_trees


def get_qos_preset_profile(qos_profile):
    """
    Get qos preset for enum value
    """
    if qos_profile[0] == 'parameters':
        return QoSPresetProfiles.PARAMETERS.value
    elif qos_profile[0] == 'parameter_events':
        return QoSPresetProfiles.PARAMETER_EVENTS.value
    elif qos_profile[0] == 'sensor_data':
        return QoSPresetProfiles.SENSOR_DATA.value
    elif qos_profile[0] == 'service_default':
        return QoSPresetProfiles.SERVICE_DEFAULT.value  # pylint: disable=no-member
    elif qos_profile[0] == 'system_default':
        return QoSPresetProfiles.SYSTEM_DEFAULT.value
    else:
        raise ValueError(f"Invalid qos_profile: {qos_profile}")


def get_comparison_operator(operator_val):  # pylint: disable=too-many-return-statements
    """
    Get comparison operator for enum value
    """
    if operator_val[0] == 'lt':
        return operator.lt
    elif operator_val[0] == 'le':
        return operator.le
    elif operator_val[0] == 'eq':
        return operator.eq
    elif operator_val[0] == 'ne':
        return operator.ne
    elif operator_val[0] == 'ge':
        return operator.ge
    elif operator_val[0] == 'gt':
        return operator.gt
    else:
        raise ValueError(f"Invalid comparison_operator: {operator_val}")


def get_clearing_policy(clearing_policy):
    """
    Get clearing policy for enum value
    """
    if clearing_policy[0] == 'on_initialise':
        return py_trees.common.ClearingPolicy.ON_INITIALISE
    elif clearing_policy[0] == 'on_success':
        return py_trees.common.ClearingPolicy.ON_SUCCESS
    elif clearing_policy[0] == 'never':
        return py_trees.common.ClearingPolicy.NEVER
    else:
        raise ValueError(f"Invalid clearing_policy: {clearing_policy}")
