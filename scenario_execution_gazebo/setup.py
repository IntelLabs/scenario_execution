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
from setuptools import setup

PACKAGE_NAME = 'scenario_execution_gazebo'

setup(
    name=PACKAGE_NAME,
    version='0.0.0',
    packages=[PACKAGE_NAME],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + PACKAGE_NAME]),
        ('share/' + PACKAGE_NAME, ['package.xml']),
        (os.path.join('share', PACKAGE_NAME, 'scenarios'), glob('scenarios/*.osc')),
        (os.path.join('share', PACKAGE_NAME, 'launch'), glob('launch/*launch.py')),
        (os.path.join('share', PACKAGE_NAME, 'params'), glob('params/*.yaml')),
        (os.path.join('share', PACKAGE_NAME, 'models'), glob('models/*.sdf*')),
        (os.path.join('share', PACKAGE_NAME, 'models'), glob('models/*.dae*')),
        (os.path.join('share', PACKAGE_NAME, 'models'), glob('models/*.png*')),
        (os.path.join('share', PACKAGE_NAME, 'worlds'), glob('worlds/*.world')),
        (os.path.join('share', PACKAGE_NAME, 'worlds'), glob('worlds/*.model'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Intel Labs',
    maintainer_email='scenario-execution@intel.com',
    description='Package for scenario execution testing in gazebo',
    license='Apache License 2.0',
    tests_require=['pytest'],
    include_package_data=True,
    entry_points={
        'scenario_execution.action_plugins': [
            'amr_object.spawn = scenario_execution_gazebo.gazebo_spawn_actor:GazeboSpawnActor',
            'amr_object.spawn_and_move = scenario_execution_gazebo.gazebo_spawn_moving_actor:GazeboSpawnMovingActor',
            'actor_exists = scenario_execution_gazebo.gazebo_actor_exists:GazeboActorExists',
            'amr_object.delete = scenario_execution_gazebo.gazebo_delete_actor:GazeboDeleteActor',
            'wait_for_sim = scenario_execution_gazebo.gazebo_wait_for_sim:GazeboWaitForSim',
        ],
        'scenario_execution.osc_libraries': [
            'gazebo = '
            'scenario_execution_gazebo.get_osc_library:get_gazebo_library',
        ]
    },
)
