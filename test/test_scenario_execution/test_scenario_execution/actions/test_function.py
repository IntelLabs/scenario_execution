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

import os
import py_trees


class TestFunction(py_trees.behaviour.Behaviour):

    # def update(self) -> py_trees.common.Status:
    #     if os.path.isfile(self.file_name):
    #         self.feedback_message = f"File '{self.file_name}' exists"  # pylint: disable= attribute-defined-outside-init
    #         return py_trees.common.Status.SUCCESS
    #     else:
    #         self.feedback_message = f"File '{self.file_name}' does not exist"  # pylint: disable= attribute-defined-outside-init
    #         return py_trees.common.Status.FAILURE
