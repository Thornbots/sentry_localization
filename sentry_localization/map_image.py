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

"""OccupancyGrid to nav2 map_server .pgm/.yaml, no rclpy (map_autosaver)."""
import math
import os

import numpy as np

FREE_THRESH = 0.196  # nav2 Jazzy map_saver defaults
OCCUPIED_THRESH = 0.65


def to_pgm(data, width, height):
    """P5 bytes as nav2 map_saver's trinary mode writes them, top row = max y."""
    cells = np.asarray(data, dtype=np.int16).reshape(height, width)[::-1]
    img = np.full(cells.shape, 205, dtype=np.uint8)
    known = (cells >= 0) & (cells <= 100)
    img[known & (cells <= round(FREE_THRESH * 100))] = 254
    img[known & (cells >= round(OCCUPIED_THRESH * 100))] = 0
    return f'P5\n{width} {height}\n255\n'.encode() + img.tobytes()


def to_yaml(image, resolution, x, y, qz, qw):
    """map_server yaml for an image next to it; origin yaw from the quaternion."""
    yaw = 2.0 * math.atan2(qz, qw)
    return (f'image: {image}\nmode: trinary\nresolution: {resolution:.6g}\n'
            f'origin: [{x:.6g}, {y:.6g}, {yaw:.6g}]\nnegate: 0\n'
            f'occupied_thresh: {OCCUPIED_THRESH}\nfree_thresh: {FREE_THRESH}\n')


def write_atomic(path, content):
    """Write via a temp file and rename, so a power cut leaves the old file."""
    tmp = path + '.tmp'
    with open(tmp, 'wb') as f:
        f.write(content if isinstance(content, bytes) else content.encode())
        f.flush()
        os.fsync(f.fileno())
    os.replace(tmp, path)
