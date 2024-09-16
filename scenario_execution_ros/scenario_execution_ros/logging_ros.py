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

""" Logger Class for ROS"""

import rclpy
from scenario_execution.utils.logging import BaseLogger


class RosLogger(BaseLogger):
    """
    Class for logger for ROS scenario execution

    Args:
        name [str]: name of the logger
    """

    def __init__(self, name: str, debug=False):
        super().__init__(name, debug)
        if debug:
            rclpy.logging.set_logger_level(name, rclpy.logging.LoggingSeverity.DEBUG)
        self.logger = rclpy.logging.get_logger(name)

    def info(self, msg: str):
        """
        Log info in ROS2

        Args:
            msg [str]: msg to print
        """
        self.logger.info(msg)

    def debug(self, msg: str):
        """
        Log debug info in ROS2

        Args:
            msg [str]: msg to print
        """
        self.logger.debug(msg)

    def warning(self, msg: str):
        """
        Log warning in ROS2

        Args:
            msg [str]: msg to print
        """
        self.logger.warning(msg)

    def error(self, msg: str):
        """
        Log error in ROS2

        Args:
            msg [str]: msg to print
        """
        self.logger.error(msg)
