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


import grpc

from kube_bullet.grpc_kube_bullet import kube_bullet_grpc_pb2_grpc


class _Singleton:
    _instance = None

    def __init__(self) -> None:
        self._insecure_port = 50051
        self.channel = grpc.insecure_channel(f"0.0.0.0:{self._insecure_port}")
        self.stub = kube_bullet_grpc_pb2_grpc.KubeBulletInterfaceStub(self.channel)


def KubeBulletClient():  # pylint: disable=C0103
    if _Singleton._instance is None:  # pylint: disable=protected-access
        _Singleton._instance = _Singleton()  # pylint: disable=protected-access
    return _Singleton._instance  # pylint: disable=protected-access
