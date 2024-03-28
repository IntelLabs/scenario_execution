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

""" Marker handler"""

from visualization_msgs.msg import Marker


class MarkerHandler(object):
    """
    Class for managing markers

    Args:
        name [str]: name of the logger
    """

    def __init__(self, node):
        self.node = node
        self.markers = []
        self.marker_publisher = self.node.create_publisher(Marker, '/scenario_marker', 5)

    def add_marker(self, marker: Marker):
        marker.header.frame_id = 'map'
        marker.action = marker.ADD
        self.markers.append(marker)

        self.marker_publisher.publish(marker)

        return self.markers.index(marker)

    def get_marker(self, marker_id):
        return self.markers[marker_id]

    def remove_markers(self, marker_id_list):
        for marker_id in marker_id_list:
            if marker_id is not None and marker_id < len(self.markers):
                marker = self.markers[marker_id]
                marker.action = marker.DELETE
                self.marker_publisher.publish(marker)

    def update_marker(self, marker_id, marker):
        self.markers[marker_id] = marker
        self.markers[marker_id].id = marker_id
        self.markers[marker_id].header.frame_id = 'map'
        self.markers[marker_id].action = marker.MODIFY

        self.marker_publisher.publish(self.markers[marker_id])
