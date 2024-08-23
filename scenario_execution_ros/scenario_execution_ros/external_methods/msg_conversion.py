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
from geometry_msgs.msg import PoseWithCovarianceStamped
import json
from rosidl_runtime_py.convert import message_to_ordereddict

from transforms3d.euler import quat2euler

def to_dict(in_value, member_name = ""):
    target = in_value
    if member_name:
        splitted_member = member_name.split('.')
        for mem in splitted_member:
            if not hasattr(target, mem):
                raise ValueError(f"Member '{mem}' not found in '{target}")
            target = getattr(target, mem)
    return json.loads(json.dumps(message_to_ordereddict(target)))

def to_pose3d(in_value):
    if isinstance(in_value, PoseWithCovarianceStamped):
        pose3d = dict()
        pose3d["position"] = to_dict(in_value.pose.pose.position)
        roll, pitch, yaw = quat2euler([in_value.pose.pose.orientation.w, in_value.pose.pose.orientation.x, in_value.pose.pose.orientation.y, in_value.pose.pose.orientation.z])
        pose3d["orientation"] = {'roll': roll, 'pitch': pitch, 'yaw': yaw}
        return pose3d
    else:
        raise ValueError(f"to_pose3d not implemented for type {type(in_value)}")
