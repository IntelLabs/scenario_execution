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


class OSC2ParsingError(Exception):
    """
    Error class for OSC2 parser
    """

    def __init__(self, msg: str, context, *args) -> None:
        super().__init__(*args)
        self.msg = msg
        if isinstance(context, ParserRuleContext):
            self.line = context.start.line
            self.column = context.start.column
            self.context = context.getText()
            self.filename = ""
        else:
            self.line = context[0]
            self.column = context[1]
            self.context = context[2]
            self.filename = context[3]

    def __str__(self) -> str:
        return self.msg