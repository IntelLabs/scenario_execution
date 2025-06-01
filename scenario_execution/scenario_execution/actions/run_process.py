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

import py_trees  # pylint: disable=import-error
import subprocess  # nosec B404
from threading import Thread
from collections import deque
import signal
from scenario_execution.actions.base_action import BaseAction
import os


class RunProcess(BaseAction):
    """
    Class to execute an process.
    """

    def __init__(self):
        super().__init__()
        self.command = None
        self.wait_for_shutdown = None
        self.shutdown_timeout = None
        self.shutdown_signal = None
        self.executed = False
        self.process = None
        self.log_stdout_thread = None
        self.log_stderr_thread = None
        self.output = deque()

    def execute(self, command=None, wait_for_shutdown=True, shutdown_timeout=10, shutdown_signal=("", signal.SIGTERM)):
        self.command = command.split(" ") if isinstance(command, str) else command
        self.wait_for_shutdown = wait_for_shutdown
        self.shutdown_timeout = shutdown_timeout
        self.shutdown_signal = shutdown_signal[1]
        self.executed = False

    def update(self) -> py_trees.common.Status:
        """
        Start/monitor process

        return:
            py_trees.common.Status
        """
        if not self.executed:
            self.executed = True
            try:
                self.process = subprocess.Popen(
                    self.command,
                    preexec_fn=os.setsid, # run in a new process group
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                )
            except Exception as e:  # pylint: disable=broad-except
                self.logger.error(str(e))
                return py_trees.common.Status.FAILURE

            self.feedback_message = f"Executing '{self.command}'"  # pylint: disable= attribute-defined-outside-init
            self.logger.debug(f"Executing '{self.command}'")
            self.on_executed()

            def log_output(out, log_fct, buffer):
                try:
                    for line in iter(out.readline, b''):
                        msg = line.decode().strip()
                        if log_fct:
                            log_fct(msg)
                        buffer.append(msg)
                    out.close()
                except ValueError:
                    pass
                except Exception as e:  # pylint: disable=broad-except
                    self.logger.error(f"Error while logging output: {e}")
            self.log_stdout_thread = Thread(target=log_output, args=(
                self.process.stdout, self.get_logger_stdout(), self.output))
            self.log_stdout_thread.daemon = True  # die with the program
            self.log_stdout_thread.start()

            self.log_stderr_thread = Thread(target=log_output, args=(
                self.process.stderr, self.get_logger_stderr(), self.output))
            self.log_stderr_thread.daemon = True  # die with the program
            self.log_stderr_thread.start()

        if self.process is None:
            self.process = None
            return py_trees.common.Status.FAILURE

        ret = self.process.poll()

        if ret is None:
            return self.check_running_process()
        else:
            return self.on_process_finished(ret)

    def get_logger_stdout(self):
        """
        get logger for stderr messages
        """
        return self.logger.info

    def get_logger_stderr(self):
        """
        get logger for stderr messages
        """
        return self.logger.error

    def check_running_process(self):
        """
        hook to check running process

        return:
            py_trees.common.Status
        """
        if self.wait_for_shutdown:
            return py_trees.common.Status.RUNNING
        else:
            return py_trees.common.Status.SUCCESS

    def on_process_finished(self, ret):
        """
        hook to check finished process

        return:
            py_trees.common.Status
        """
        if ret == 0:
            self.feedback_message = f"Successfully executed '{self.command}'"  # pylint: disable= attribute-defined-outside-init
            return py_trees.common.Status.SUCCESS
        else:
            self.feedback_message = f"Execution of '{self.command}' failed with {ret}"  # pylint: disable= attribute-defined-outside-init
            return py_trees.common.Status.FAILURE

    def on_executed(self):
        """
        hook for subclassed
        """
        pass

    def set_command(self, command):
        self.command = command

    def get_command(self):
        return self.command

    def shutdown(self):
        if self.process is None:
            return

        ret = self.process.poll()
        if ret is None:
            # kill running process
            self.logger.info(f'Sending {signal.Signals(self.shutdown_signal).name} to process...')
            pgid = os.getpgid(self.process.pid)
            os.killpg(pgid, self.shutdown_signal)
            if self.process.poll():
                self.logger.info(f"Waiting {self.shutdown_timeout}s for process to finish...")
                self.process.wait(self.shutdown_timeout)
                self.logger.info('Sending SIGKILL to process...')
                os.killpg(pgid, signal.SIGKILL)
                self.process.wait()
            self.logger.info('Process finished.')
