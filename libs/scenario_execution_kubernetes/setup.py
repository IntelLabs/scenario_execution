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

from glob import glob
import os
from setuptools import find_namespace_packages, setup

PACKAGE_NAME = 'scenario_execution_kubernetes'

setup(
    name=PACKAGE_NAME,
    version='1.2.0',
    packages=find_namespace_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + PACKAGE_NAME]),
        ('share/' + PACKAGE_NAME, ['package.xml']),
        (os.path.join('share', PACKAGE_NAME, 'scenarios'), glob('scenarios/*.osc')),
        (os.path.join('share', PACKAGE_NAME, 'launch'), glob('launch/*launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    include_package_data=True,
    maintainer='Intel Labs',
    maintainer_email='scenario-execution@intel.com',
    description='Robotics Scenario Execution Kubernetes Addon',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'scenario_execution.actions': [
            'kubernetes_create_from_yaml = scenario_execution_kubernetes.kubernetes_create_from_yaml:KubernetesCreateFromYaml',
            'kubernetes_delete = scenario_execution_kubernetes.kubernetes_delete:KubernetesDelete',
            'kubernetes_patch_network_policy = scenario_execution_kubernetes.kubernetes_patch_network_policy:KubernetesPatchNetworkPolicy',
            'kubernetes_patch_pod = scenario_execution_kubernetes.kubernetes_patch_pod:KubernetesPatchPod',
            'kubernetes_pod_exec = scenario_execution_kubernetes.kubernetes_pod_exec:KubernetesPodExec',
            'kubernetes_wait_for_network_policy_status = scenario_execution_kubernetes.kubernetes_wait_for_network_policy_status:KubernetesWaitForNetworkPolicyStatus',
            'kubernetes_wait_for_pod_status = scenario_execution_kubernetes.kubernetes_wait_for_pod_status:KubernetesWaitForPodStatus',
        ],
        'scenario_execution.osc_libraries': [
            'kubernetes = scenario_execution_kubernetes.get_osc_library:get_kubernetes_library',
        ]
    },
)
