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

""" Setup python package """
from glob import glob
import os
from setuptools import find_packages, setup

PACKAGE_NAME = 'scenario_execution_openrmf'

setup(
    name=PACKAGE_NAME,
    version='0.0.0',
    packages=find_packages(),
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
    maintainer='TODO',
    maintainer_email='TODO@TODO.com',
    description='Robotics Scenario Execution Open RMF Addon',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'scenario_execution.action_plugins': [
            'door_request = scenario_execution_openrmf.door_request:DoorRequest',
        ],
        'scenario_execution.osc_libraries': [
            'openrmf = '
            'scenario_execution_openrmf.get_osc_library:get_openrmf_library',
        ]
    },
)
