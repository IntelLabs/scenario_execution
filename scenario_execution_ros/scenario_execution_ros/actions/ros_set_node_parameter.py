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

from .ros_service_call import RosServiceCall
from rcl_interfaces.msg import ParameterType


class RosSetNodeParameter(RosServiceCall):
    """
    class for setting a node parameter
    """

    def __init__(self, node_name: str, parameter_name: str, parameter_value: str):
        self.node_name = node_name
        self.parameter_name = parameter_name
        self.parameter_value = parameter_value
        service_name = node_name + '/set_parameters'
        if not service_name.startswith('/'):
            service_name = '/' + service_name

        parameter_type = ParameterType.PARAMETER_STRING
        parameter_assign_name = 'string_value'
        if parameter_value.lower() == 'true' or parameter_value.lower() == 'false':
            parameter_type = ParameterType.PARAMETER_BOOL
            parameter_assign_name = 'bool_value'
        elif parameter_value.isdigit():
            parameter_type = ParameterType.PARAMETER_INTEGER
            parameter_assign_name = 'integer_value'
        elif RosSetNodeParameter.is_float(parameter_value):
            parameter_type = ParameterType.PARAMETER_DOUBLE
            parameter_assign_name = 'double_value'
        if parameter_value.startswith('['):
            try:
                vals = literal_eval(parameter_value)
            except SyntaxError as e:
                self.logger.error(f"Could not parse parameter_value: {parameter_value}: {e}")
                pass
            if vals:
                if vals[0].lower() == 'true' or vals[0].lower() == 'false':
                    parameter_type = ParameterType.PARAMETER_BOOL_ARRAY
                    parameter_assign_name = 'bool_array_value'
                elif vals[0].isdigit():
                    parameter_type = ParameterType.PARAMETER_INTEGER_ARRAY
                    parameter_assign_name = 'integer_array_value'
                elif RosSetNodeParameter.is_float(vals[0]):
                    parameter_type = ParameterType.PARAMETER_DOUBLE_ARRAY
                    parameter_assign_name = 'double_array_value'
                else:
                    parameter_type = ParameterType.PARAMETER_STRING_ARRAY
                    parameter_assign_name = 'string_array_value'

        super().__init__(service_name=service_name,
                         service_type='rcl_interfaces.srv.SetParameters',
                         data='{ "parameters": [{ "name": "' + parameter_name + '", "value": { "type": ' + str(parameter_type) + ', "' + parameter_assign_name + '": ' + parameter_value + '}}]}', transient_local=False)

    def execute(self, node_name: str, parameter_name: str, parameter_value: str):   # pylint: disable=arguments-differ,arguments-renamed
        if self.node_name != node_name or self.parameter_name != parameter_name or self.parameter_value != parameter_value:
            raise ValueError("node_name, parameter_name and parameter_value are not changeable during runtime.")

    @staticmethod
    def is_float(element: any) -> bool:
        """
        test for float type
        """

        try:
            float(element)
            return True
        except ValueError:
            return False
