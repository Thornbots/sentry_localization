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

"""Unit tests for map_pose_core, no rclpy: `pytest test/test_map_pose_core.py`."""
import math
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from sentry_localization.map_pose_core import compose, rotate_cov_xy  # noqa: E402


def test_identity_map_odom_passes_pose_through():
    assert compose((0.0, 0.0, 0.0), (1.0, -2.0, 0.3)) == pytest.approx((1.0, -2.0, 0.3))


def test_translation_adds():
    assert compose((0.5, 0.25, 0.0), (1.0, 1.0, 0.0)) == pytest.approx((1.5, 1.25, 0.0))


def test_rotation_turns_odom_position_then_translates():
    x, y, yaw = compose((1.0, 0.0, math.pi / 2), (2.0, 0.0, 0.0))
    assert (x, y, yaw) == pytest.approx((1.0, 2.0, math.pi / 2))


def test_yaw_wraps():
    assert compose((0.0, 0.0, 3.0), (0.0, 0.0, 3.0))[2] == pytest.approx(6.0 - 2 * math.pi)


def test_cov_rotation_quarter_turn_swaps_axes():
    (xx, xy), (yx, yy) = rotate_cov_xy(((4.0, 0.0), (0.0, 1.0)), math.pi / 2)
    assert (xx, xy, yx, yy) == pytest.approx((1.0, 0.0, 0.0, 4.0))


def test_cov_rotation_keeps_trace_and_symmetry():
    (xx, xy), (yx, yy) = rotate_cov_xy(((3.0, 0.5), (0.5, 2.0)), 0.7)
    assert xx + yy == pytest.approx(5.0)
    assert xy == yx
    assert xx * yy - xy * xy == pytest.approx(3.0 * 2.0 - 0.25)
