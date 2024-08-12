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

from ast import literal_eval
from .kubernetes_base_action import KubernetesBaseAction


class KubernetesPatchPod(KubernetesBaseAction):

    def __init__(self, namespace: str, target: str, body: str, within_cluster: bool):
        super().__init__(namespace, within_cluster)
        self.target = target
        self.body = None

    def execute(self, namespace: str, target: str, body: str, within_cluster: bool):  # pylint: disable=arguments-differ
        super().execute(namespace, within_cluster)
        self.target = target
        trimmed_data = body.encode('utf-8').decode('unicode_escape')
        try:
            self.body = literal_eval(trimmed_data)
        except ValueError as e:
            raise ValueError(f"Could not parse body '{trimmed_data}': {e}") from e

    def kubernetes_call(self):
        self.feedback_message = f"Requested patching '{self.target}' in namespace '{self.namespace}'"  # pylint: disable= attribute-defined-outside-init
        return self.client.patch_namespaced_pod(self.target, body=self.body, namespace=self.namespace, async_req=True)
