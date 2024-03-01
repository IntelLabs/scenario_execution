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

""" Setup file for package example_simulation_nav2 """
from glob import glob
import os
from setuptools import setup

PACKAGE_NAME = 'example_nav2'

setup(
    name=PACKAGE_NAME,
    version='0.0.0',
    packages=[PACKAGE_NAME],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + PACKAGE_NAME]),
        ('share/' + PACKAGE_NAME, ['package.xml']),
        (os.path.join('share', PACKAGE_NAME, 'scenarios'), glob('scenarios/*.osc'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Intel Labs',
    maintainer_email='scenario-execution@intel.com',
    description='Scenario Execution Example for Nav2',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
    },
)
