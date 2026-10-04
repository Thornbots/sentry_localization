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

"""Unit tests for map_image, no rclpy: `pytest test/test_map_image.py`."""
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from sentry_localization.map_image import to_pgm, to_yaml, write_atomic  # noqa: E402


def test_pgm_trinary_values_and_row_order():
    # Row 0 of the grid is min y, so it is the image's last row.
    data = [-1, 0, 20, 21,
            64, 65, 100, 101]
    pgm = to_pgm(data, 4, 2)
    header = b'P5\n4 2\n255\n'
    assert pgm.startswith(header)
    assert list(pgm[len(header):]) == [205, 0, 0, 205,
                                       205, 254, 254, 205]


def test_yaml_origin_yaw_from_quaternion():
    y = to_yaml('map.pgm', 0.05, -1.5, 2.0, math.sin(0.25), math.cos(0.25))
    assert 'image: map.pgm\n' in y
    assert 'mode: trinary\n' in y
    assert 'resolution: 0.05\n' in y
    assert 'origin: [-1.5, 2, 0.5]\n' in y


def test_write_atomic_replaces_and_leaves_no_tmp(tmp_path):
    p = str(tmp_path / 'map.yaml')
    write_atomic(p, 'old')
    write_atomic(p, b'new')
    assert open(p).read() == 'new'
    assert os.listdir(tmp_path) == ['map.yaml']
