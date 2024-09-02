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

import sensor_msgs.msg
    
def generate_launch_params(in_value):
    if isinstance(in_value, sensor_msgs.msg.JointState):
        key_value_list = []
        for name, position in zip(in_value.name, in_value.position):
            key_value_list.append({'key': name, 'value': position})
        return key_value_list
    elif isinstance(in_value, dict) and in_value == {}:
        return in_value
    else:
        raise ValueError(f"to_joint_state_key_value_list not implemented for type {type(in_value)}")
    