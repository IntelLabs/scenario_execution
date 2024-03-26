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

"""Setup python package"""
from glob import glob
import os
from setuptools import find_namespace_packages, setup

PACKAGE_NAME = 'scenario_execution_base'

setup(
    name=PACKAGE_NAME,
    version='1.0.0',
    packages=find_namespace_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + PACKAGE_NAME]),
        ('share/' + PACKAGE_NAME, ['package.xml']),
        (os.path.join('share', PACKAGE_NAME, 'launch'), glob('launch/*launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    include_package_data=True,
    maintainer='Intel Labs',
    maintainer_email='scenario-execution@intel.com',
    description='Robotics Scenario Execution',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'scenario_execution_base = scenario_execution_base.scenario_execution:main',
        ],
        'scenario_execution.actions': [
            'log = scenario_execution_base.actions.log:Log',
            'run_process = scenario_execution_base.actions.run_process:RunProcess',
        ],
        'scenario_execution.osc_libraries': [
            'helpers = scenario_execution_base.get_osc_library:get_helpers_library',
            'standard = scenario_execution_base.get_osc_library:get_standard_library',
            'standard.base = scenario_execution_base.get_osc_library:get_standard_base_library',
            'robotics = scenario_execution_base.get_osc_library:get_robotics_library',
        ]
    },
)
