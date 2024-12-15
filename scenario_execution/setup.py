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
from pathlib import Path
from glob import glob
import os
from setuptools import find_namespace_packages, setup

PACKAGE_NAME = 'scenario_execution'

# read the contents of the README file
this_directory = Path(__file__).parent
try:
    LONG_DESCRIPTION = (this_directory / "README.md").read_text()
except:  # pylint: disable=W0702
    # in case we do colcon build --symlink-install, wo do not need the
    # description
    LONG_DESCRIPTION = ''

setup(
    name=PACKAGE_NAME,
    version='1.3.0',
    packages=find_namespace_packages(exclude=['test*']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + PACKAGE_NAME]),
        ('share/' + PACKAGE_NAME, ['package.xml']),
        (os.path.join('share', PACKAGE_NAME, 'launch'), glob('launch/*launch.py'))
    ],
    install_requires=[
        'setuptools',
        'antlr4-python3-runtime==4.9.2',
        'pyyaml==6.0.1',
        'py-trees==2.2.1'
    ],
    zip_safe=True,
    include_package_data=True,
    maintainer='Intel Labs',
    maintainer_email='scenario-execution@intel.com',
    url='https://github.com/IntelLabs/scenario_execution',
    project_urls={
        "Homepage": "https://github.com/IntelLabs/scenario_execution",
        "Documentation": "https://github.com/IntelLabs/scenario_execution",
        "Issues": "https://github.com/IntelLabs/scenario_execution/issues",
    },
    description='Scenario Execution for Robotics',
    long_description=LONG_DESCRIPTION,
    long_description_content_type='text/markdown',
    license='Apache License 2.0',
    classifiers=[
        "Programming Language :: Python :: 3",
    ],
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'scenario_execution = scenario_execution.scenario_execution_base:main',
        ],
        'scenario_execution.actions': [
            'increment = scenario_execution.actions.increment:Increment',
            'decrement = scenario_execution.actions.decrement:Decrement',
            'log = scenario_execution.actions.log:Log',
            'run_process = scenario_execution.actions.run_process:RunProcess',
        ],
        'scenario_execution.osc_libraries': [
            'helpers = scenario_execution.get_osc_library:get_helpers_library',
            'standard = scenario_execution.get_osc_library:get_standard_library',
            'types = scenario_execution.get_osc_library:get_types_library',
            'robotics = scenario_execution.get_osc_library:get_robotics_library',
        ]
    },
)
