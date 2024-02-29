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
import os
from glob import glob
from setuptools import find_packages, setup

PACKAGE_NAME = 'scenario_status'

setup(
    name=PACKAGE_NAME,
    version='0.0.1',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + PACKAGE_NAME]),
        ('share/' + PACKAGE_NAME, ['package.xml']),
        (os.path.join('share', PACKAGE_NAME), glob('scenario_status/*.py')),
        (os.path.join('share', PACKAGE_NAME), glob('params/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fmirus',
    maintainer_email='florian.mirus@intel.com',
    description='Simple node to call a service to publish the py-trees-\
        behaviour tree to a topic, then subscribe to that topic and publish \
        changes in behaviour states as strings at the time they are \
        happening.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'scenario_status_node = scenario_status.scenario_status_node:main'
        ],
    },
)
