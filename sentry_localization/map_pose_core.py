# Copyright 2026 Thornbots
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Planar map->root composition for map_pose_publisher.py (no rclpy import)."""
import math


def compose(map_odom, odom_root):
    """Return map->root (x, y, yaw) from map->odom and odom->root, each (x, y, yaw)."""
    mx, my, myaw = map_odom
    ox, oy, oyaw = odom_root
    c, s = math.cos(myaw), math.sin(myaw)
    yaw = math.atan2(math.sin(myaw + oyaw), math.cos(myaw + oyaw))
    return mx + c * ox - s * oy, my + s * ox + c * oy, yaw


def rotate_cov_xy(cov_xy, yaw):
    """Rotate a 2x2 xy covariance ((xx, xy), (yx, yy)) by yaw: R C R^T."""
    (a, b), (_, d) = cov_xy
    c, s = math.cos(yaw), math.sin(yaw)
    xx = c * c * a - 2 * c * s * b + s * s * d
    yy = s * s * a + 2 * c * s * b + c * c * d
    xy = c * s * (a - d) + (c * c - s * s) * b
    return (xx, xy), (xy, yy)
