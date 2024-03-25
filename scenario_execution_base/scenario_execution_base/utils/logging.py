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


class BaseLogger(object):
    """
    Base class for logger for scenario execution
    For different middleware that does not have logger, inherit from this class
    and override the virtual methods

    Args:
        name [str]: name of the logger
    """

    def __init__(self, name: str, debug) -> None:
        self.name = name
        self.log_debug = debug

    def info(self, msg: str) -> None:
        """
        Virtual method to log info

        Args:
            msg [str]: msg to print
        """
        raise NotImplementedError

    def debug(self, msg: str) -> None:
        """
        virtual method to log debug info

        Args:
            msg [str]: msg to print
        """
        raise NotImplementedError

    def warning(self, msg: str) -> None:
        """
        Virtual method to log warning

        Args:
            msg [str]: msg to print
        """
        raise NotImplementedError

    def error(self, msg: str) -> None:
        """
        Virtual method to log error

        Args:
            msg [str]: msg to print
        """
        raise NotImplementedError


class Logger(BaseLogger):
    """
    Class for logger for scenario execution

    Args:
        name [str]: name of the logger
    """

    def info(self, msg: str) -> None:
        """
        Print a info with logger name and "[INFO]" in the front.

        Args:
            msg [str]: msg to print
        """
        print(f'[{self.name}] [INFO] {msg}')

    def debug(self, msg: str) -> None:
        """
        Print debug info with logger name and "[DEBUG]" in the front.

        Args:
            msg [str]: msg to print
        """
        if self.log_debug:
            print(f'[{self.name}] [DEBUG] {msg}')

    def warning(self, msg: str) -> None:
        """
        Print a warning in yellow color with logger name and "[WARN]" in the front.

        Args:
            msg [str]: msg to print
        """
        print(f'\033[33m[{self.name}] [WARN] {msg}\033[0m')

    def error(self, msg: str) -> None:
        """
        Print a warning in red color with logger name and "[ERROR]" in the front.

        Args:
            msg [str]: msg to print
        """
        print(f'\033[31m[{self.name}] [ERROR] {msg}\033[0m')
