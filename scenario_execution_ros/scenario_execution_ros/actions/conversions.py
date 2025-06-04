# Copyright (C) 2025 Frederik Pasch
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
import importlib
from rclpy.qos import QoSPresetProfiles, ReliabilityPolicy, DurabilityPolicy
from array import array

def get_ros_message_type(message_type_string):
    if not message_type_string:
        raise ValueError("Empty message type.")
    if '.' in message_type_string and '/' in message_type_string:
        raise ValueError("Either use '.' or '/' as separator")
    if '/' in message_type_string:
        message_type_string.replace('/', '.')
    datatype_in_list = message_type_string.split('.')
    try:
        return getattr(importlib.import_module(".".join(datatype_in_list[0:-1])), datatype_in_list[-1])
    except (ModuleNotFoundError, ValueError) as e:
        raise ValueError(f"Could not find message type {message_type_string}: {e}") from e


def get_qos_preset_profile(qos_profile): # pylint: disable=too-many-return-statements
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
    elif qos_profile[0] == 'system_default_reliable':
        profile = QoSPresetProfiles.SYSTEM_DEFAULT.value
        profile.reliability = ReliabilityPolicy.RELIABLE
        return profile
    elif qos_profile[0] == 'system_default_transient_local':
        profile = QoSPresetProfiles.SYSTEM_DEFAULT.value
        profile.durability = DurabilityPolicy.TRANSIENT_LOCAL
        return profile
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

def set_variable_if_available(msg, target_variable, member_name):
    if target_variable is not None:
        val = msg
        if member_name:
            val = getattr(msg, member_name)
            if isinstance(val, array):
                val = val.tolist()
        target_variable.set_value(val)
