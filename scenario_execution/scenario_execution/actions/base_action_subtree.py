# Copyright (C) 2025 Frederik Pasch
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

class BaseActionSubtree(py_trees.composites.Sequence):
    """
    Base class for actions that provide their own behavior tree implementation.
    
    Instead of implementing update() directly, subclasses should override create_subtree()
    to return a complete behavior tree that will be used as the action's implementation.
    
    This class acts as a wrapper that returns itself as the main behavior but delegates
    the actual work to the subtree it creates.
    """

    def __init__(self):
        self._model = None
        self.logger = None
        execute_method = getattr(self, "execute", None)
        if execute_method is not None and callable(execute_method):
            raise ValueError(
                "BaseActionSubtree derived class should not implement execute(). ")
        super().__init__(self.__class__.__name__, memory=True)

    def get_execution_args(self, child):
        raise NotImplementedError("Subclasses must implement get_execution_args()")

    def initialise(self):
        pass

    def _set_base_properities(self, name, model, logger):
        self.name = name
        self._model = model
        self.logger = logger
