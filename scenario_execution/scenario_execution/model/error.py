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


from antlr4 import ParserRuleContext


class OSC2Error(Exception):

    def __init__(self, msg: str, context, *args) -> None:
        super().__init__(*args)
        self.msg = msg
        if isinstance(context, ParserRuleContext):
            self.osc_ctx = (context.start.line, context.start.column, context.getText(), "")
        else:
            self.osc_ctx = context

    def __str__(self) -> str:
        error_str = ""
        if self.osc_ctx is not None:
            context = self.osc_ctx[2].replace('\n', '')
            error_str = f"(line: {self.osc_ctx[0]}, column: {self.osc_ctx[1]} in '{self.osc_ctx[3]}') -> {context}: "
        error_str += self.msg
        return error_str


class OSC2ParsingError(OSC2Error):
    """
    Error class for OSC2 parser
    """

    def __init__(self, msg: str, context, *args) -> None:
        if isinstance(context, ParserRuleContext):
            ctx = (context.start.line, context.start.column, context.getText(), "")
        else:
            ctx = context
        super().__init__(msg, ctx, *args)
