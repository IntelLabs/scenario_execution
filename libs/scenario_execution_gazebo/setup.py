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
from setuptools import find_namespace_packages, setup

PACKAGE_NAME = 'scenario_execution_gazebo'

setup(
    name=PACKAGE_NAME,
    version='1.1.0',
    packages=find_namespace_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + PACKAGE_NAME]),
        ('share/' + PACKAGE_NAME, ['package.xml'])
    ],
    install_requires=[
        'setuptools',
        'transforms3d==0.3.1',
        'defusedxml==0.7.1',
    ],
    zip_safe=True,
    maintainer='Intel Labs',
    maintainer_email='scenario-execution@intel.com',
    description='Scenario Execution library for Gazebo',
    license='Apache License 2.0',
    tests_require=['pytest'],
    include_package_data=True,
    entry_points={
        'scenario_execution.actions': [
            'osc_actor.relative_spawn = scenario_execution_gazebo.actions.gazebo_relative_spawn_actor:GazeboRelativeSpawnActor',
            'osc_actor.spawn = scenario_execution_gazebo.actions.gazebo_spawn_actor:GazeboSpawnActor',
            'actor_exists = scenario_execution_gazebo.actions.gazebo_actor_exists:GazeboActorExists',
            'osc_actor.delete = scenario_execution_gazebo.actions.gazebo_delete_actor:GazeboDeleteActor',
            'wait_for_sim = scenario_execution_gazebo.actions.gazebo_wait_for_sim:GazeboWaitForSim',
        ],
        'scenario_execution.osc_libraries': [
            'gazebo = '
            'scenario_execution_gazebo.get_osc_library:get_gazebo_library',
        ]
    },
)
