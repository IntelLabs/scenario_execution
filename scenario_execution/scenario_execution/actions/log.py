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
from py_trees.common import Status


class Log(py_trees.behaviour.Behaviour):
    """
    Class for logging

    Args:
        msg [str]: message to log
    """

    def __init__(self, name, msg: str):
        super().__init__(name)
        self.msg = msg
        self.published = False
        self.logger = None

    def setup(self, **kwargs) -> None:
        """
        setup before ticking

        Args:
            kwargs: arguments passed from py_trees.behaviour.Behaviour
        """
        self.logger = kwargs['logger']

    def update(self) -> py_trees.common.Status:
        """
        execute the action

        return:
            py_trees.common.Status.SUCCESS if the action is executed
        """
        if not self.published:
            self.published = True
            self.logger.info(self.msg)
        return Status.SUCCESS
