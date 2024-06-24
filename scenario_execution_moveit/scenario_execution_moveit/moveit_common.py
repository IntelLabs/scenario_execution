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

from pymoveit2 import MoveIt2, GripperInterface


class _MoveIt2InterfaceSingleton:
    _instance = None

    def __init__(self, node, joint_names, base_link_name, end_effector_name, group_name, callback_group) -> None:
        self.moveit2 = MoveIt2(
            node=node,
            joint_names=joint_names,
            base_link_name=base_link_name,
            end_effector_name=end_effector_name,
            group_name=group_name,
            callback_group=callback_group
        )

    def __getattr__(self, name):
        return getattr(self.moveit2, name)


class _GripperSingleton:
    _instance = None

    def __init__(self, node, gripper_joint_names, open_gripper_joint_positions, closed_gripper_joint_positions, gripper_group_name, callback_group, gripper_command_action_name) -> None:
        self.gripper_interface = GripperInterface(
            node=node,
            gripper_joint_names=gripper_joint_names,
            open_gripper_joint_positions=open_gripper_joint_positions,
            closed_gripper_joint_positions=closed_gripper_joint_positions,
            gripper_group_name=gripper_group_name,
            callback_group=callback_group,
            gripper_command_action_name=gripper_command_action_name
        )

    def __getattr__(self, name):
        return getattr(self.gripper_interface, name)


def MoveIt2Interface(node, joint_names, base_link_name, end_effector_name, group_name, callback_group):  # pylint: disable=C0103
    if _MoveIt2InterfaceSingleton._instance is None:  # pylint: disable=protected-access
        _MoveIt2InterfaceSingleton._instance = _MoveIt2InterfaceSingleton(  # pylint: disable=protected-access
            node, joint_names, base_link_name, end_effector_name, group_name, callback_group)
    return _MoveIt2InterfaceSingleton._instance  # pylint: disable=protected-access


def Gripper(node, gripper_joint_names, open_gripper_joint_positions, closed_gripper_joint_positions, gripper_group_name, callback_group, gripper_command_action_name):  # pylint: disable=C0103
    if _GripperSingleton._instance is None:  # pylint: disable=protected-access
        _GripperSingleton._instance = _GripperSingleton(node, gripper_joint_names, open_gripper_joint_positions,  # pylint: disable=protected-access
                                                        closed_gripper_joint_positions, gripper_group_name, callback_group, gripper_command_action_name)
    return _GripperSingleton._instance  # pylint: disable=protected-access
