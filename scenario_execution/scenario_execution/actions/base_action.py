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

class BaseAction(py_trees.behaviour.Behaviour):
    
    def __init__(self, name):
        super().__init__(name)
        self.model = None
    
    def set_model(self, model):
        self.model = model
        
    def initialise(self):
        final_args = self.model.get_resolved_value()
        
        if self.model.actor:
            final_args["associated_actor"] = self.model.actor.get_resolved_value()
            final_args["associated_actor"]["name"] = self.model.actor.name
        self.execute(**final_args)
        