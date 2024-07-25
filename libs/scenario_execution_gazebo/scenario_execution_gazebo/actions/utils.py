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

import os
import subprocess  # nosec B404
import glob
import defusedxml.ElementTree
from ament_index_python import get_package_share_directory
from pathlib import Path


class SpawnUtils(object):

    """Utils such as reading/parsing xml/sdf/urdf/xacro files for spawning
    actors"""

    def __init__(self, logger=None):
        """Initialize SpawnUtils class

        :logger: variable for using a ROS logger inside the SpawnUtils class

        """
        self.logger = logger
        self.ign_gazebo_resource_paths = []
        if "IGN_GAZEBO_RESOURCE_PATH" in os.environ:
            self.ign_gazebo_resource_paths = os.environ["IGN_GAZEBO_RESOURCE_PATH"].split(':')

    def parse_model_file(self, model_file: str, entity_name: str, xacro_arguments: str) -> str:
        """
        Parse model file to str:

        Args:
            model_file [str]: path to the model file

        return:
            str of the content of model file in urdf format
        """
        if '://' in model_file:
            if model_file.startswith('https://fuel'):
                return '<?xml version=\\"1.0\\" ?><sdf version=\\"1.6\\"><include><uri>' + model_file + '</uri></include></sdf>'
            elif model_file.startswith('file://'):
                # specifying sdf_filename seems to be broken, therefore load sdf
                model_file = model_file.replace('file://', '', 1)
                return self.read_model_file(model_file)
            elif model_file.lower().endswith('.xacro'):
                return self.xacro_to_urdf(self.get_packaged_model_file_path(model_file), xacro_arguments)
            elif model_file.lower().endswith(('.sdf', '.urdf')):
                return self._parse_xml(
                    self.get_packaged_model_file_path(model_file),
                    entity_name)
            elif model_file.startswith('model://'):
                # search gazebo model paths for model
                model_file = model_file.replace('model://', '', 1)
                for path in self.ign_gazebo_resource_paths:
                    if not os.path.isdir(path):
                        continue
                    subfolders = [f.path for f in os.scandir(path) if f.is_dir()]
                    for folder in subfolders:
                        basename = os.path.basename(folder)
                        if basename == model_file:
                            path = os.path.join(folder, 'model.sdf')
                            if self.logger is not None:
                                self.logger.info(
                                    f"Found model dir for {model_file}: {folder}")
                            else:
                                print(f"Found model dir for {model_file}: {folder}")

                            return self.read_model_file(path)
                self.logger.error(
                    f"Could not find {model_file} in IGN_GAZEBO_RESOURCE_PATHS")
        else:
            if self.logger is not None:
                self.logger.error(
                    f'[{entity_name}] unrecognized model file definition: {model_file}')
            else:
                print(f'[{entity_name}] unrecognized model file definition: {model_file}')
        return None

    def _parse_xml(self, xml: str, entity_name: str) -> str:
        '''
        Parse xml file to str.

        Args:
            xml [str]: path to xml file
            entity_name [str]: name of the spawned entity

        return:
            str of content of xml file
        '''
        if self.logger is not None:
            self.logger.info(f'[{entity_name}] Parsing xml: [{xml}]')
        else:
            print(f'[{entity_name}] Parsing xml: [{xml}]')
        try:
            tree = defusedxml.ElementTree.parse(xml)
            root = tree.getroot()
        except FileNotFoundError as e:
            if self.logger is not None:
                self.logger.error(f'Parsing xml {entity_name} failed: {e}')
            else:
                print(f'Parsing xml {entity_name} failed: {e}')
        return defusedxml.ElementTree.tostring(root).decode().replace("\"", "\\\"").replace("\n", "")

    @staticmethod
    def get_packaged_model_file_path(model_path):
        """
        get path of a packaged model file
        """
        model_file = model_path.split('://')
        if len(model_file) != 2:
            raise ValueError(
                f'scenario_execution_amr_plugin:spawn: Invalid model file path: {model_path}." \
                " Expected format <package-name>://<path-to-model.file>.')
        model_file_path = os.path.join(get_package_share_directory(
            model_file[0]), model_file[1])
        if not os.path.exists(model_file_path):
            raise ValueError(
                f'scenario_execution_amr_plugin:spawn: Model file not existing: {model_file_path}.')
        return model_file_path

    def xacro_to_urdf(self, path_to_xacro, xacro_arguments) -> str:
        """
        function to convert a xacro file

        Args:
            path_to_xacro [str]: path to xacro file
            entity_name [str]: name of the spawned entity

        return:
            str of content of coverted xacro file
        """
        if self.logger is not None:
            self.logger.info(
                f'Parsing xacro: {path_to_xacro}, args: {xacro_arguments}')
        else:
            print(f'Parsing xacro: {path_to_xacro}, args: {xacro_arguments}')

        arg_list = xacro_arguments.split(',')
        try:
            with subprocess.Popen(['xacro', path_to_xacro] + arg_list, stdout=subprocess.PIPE,
                                  stderr=subprocess.PIPE, text=True) as process:
                stdout, stderr = process.communicate()
                if stderr:
                    if self.logger is not None:
                        self.logger.warning(f'Parsing xacro has stderr: {stderr}')
                    else:
                        print(f'Parsing xacro has stderr: {stderr}')
        except subprocess.CalledProcessError as e:
            stdout = None
            if self.logger is not None:
                self.logger.error(f'Parsing xacro failed: {e.output}')
            else:
                print(f'Parsing xacro failed: {e.output}')
        return stdout.replace("\"", "\\\"").replace("\n", "")

    def read_model_file(self, model_file):
        '''
        read model from file
        '''
        try:
            f = open(model_file, 'r')
            xml = f.read().replace("\"", "\\\"").replace("\n", "")
            f.close()
            return xml
        except FileNotFoundError as e:
            if self.logger is not None:
                self.logger.error(f'Parsing xml {model_file} failed: {e}')
            else:
                print(f'Parsing xml {model_file} failed: {e}')
            return None

    def find_mesh(self, mesh_name: str, entity_name: str) -> str:
        """find the mesh given by a certain name.

        :mesh_name: Name of the mesh
        :returns: file path to the mesh

        """
        if '://' in mesh_name:
            if mesh_name.startswith('https://fuel'):
                return mesh_name
            elif mesh_name.startswith('file://'):
                # make sure we have an absolute mesh filepath
                try:
                    mesh_file = str(Path(mesh_name.replace('file://', '', 1)).resolve())
                except FileNotFoundError as e:
                    if self.logger is not None:
                        self.logger.error(f'Could not find mesh {mesh_name} for {entity_name}: {e}')
                    else:
                        print(f'Could not find mesh {mesh_name} for {entity_name}: {e}')
                return mesh_file
            elif mesh_name.startswith('model://'):
                # search gazebo model paths for mesh
                mesh_file = mesh_name.replace('model://', '', 1)
                if not mesh_file.endswith('.dae'):
                    mesh_file += '.dae'

                for path in self.ign_gazebo_resource_paths:
                    if not os.path.isdir(path):
                        continue
                    for filename in glob.iglob(path + '**/**', recursive=True):
                        path = Path(filename)
                        if str(path.name) == mesh_file:
                            if self.logger is not None:
                                self.logger.info(
                                    f"Found mesh dir for {mesh_file}: {path.parent}")
                            else:
                                print(f"Found mesh dir for {mesh_file}: {path.parent}")
                            return str(path.resolve())

                self.logger.error(
                    f"Could not find {mesh_name} in IGN_GAZEBO_RESOURCE_PATHS")
            elif mesh_name.endswith('.dae'):
                try:
                    mesh_file = self.get_packaged_model_file_path(mesh_name)
                    return mesh_file
                except FileNotFoundError as e:
                    if self.logger is not None:
                        self.logger.error(f'Could not find mesh {mesh_name} for {entity_name}: {e}')
                    else:
                        print(f'Could not find mesh {mesh_name} for {entity_name}: {e}')

        else:
            if self.logger is not None:
                self.logger.error(
                    f'[{entity_name}] unrecognized model file definition: {mesh_name}')
            else:
                print(f'[{entity_name}] unrecognized model file definition: {mesh_name}')
        return None

    def get_env():  # pylint: disable=no-method-argument
        if "ROS_DISTRO" not in os.environ:
            raise ValueError("ROS_DISTRO not set.")

        if os.environ["ROS_DISTRO"] == "humble":
            return 'ign', 'ignition'
        else:
            return 'gz', 'gz'
