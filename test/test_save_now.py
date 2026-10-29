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

"""Exercise the installed saver with a fake serialization backend; no robot."""

import json
import os
from pathlib import Path
import subprocess
import time

from ament_index_python.packages import get_package_prefix
from nav_msgs.msg import OccupancyGrid
import pytest
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile
from sentry_localization.srv import SaveMap
from slam_toolbox.srv import SerializePoseGraph


@pytest.fixture
def saver(tmp_path):
    """Keep every test's DDS graph and subprocess isolated and bounded."""
    previous = os.environ.get('ROS_DOMAIN_ID')
    os.environ['ROS_DOMAIN_ID'] = str(100 + os.getpid() % 100)
    rclpy.init()
    node = rclpy.create_node('save_now_test')
    behavior = {'result': 0, 'files': True, 'calls': 0, 'delay': 0.0}

    def serialize(request, response):
        behavior['calls'] += 1
        if behavior.get('collision'):
            (tmp_path / 'snapshot').mkdir()
        time.sleep(behavior['delay'])
        if behavior['files']:
            for suffix in ('.posegraph', '.data'):
                Path(request.filename + suffix).write_bytes(b'fake graph')
        response.result = behavior['result']
        return response

    service = node.create_service(
        SerializePoseGraph, '/slam_toolbox/serialize_map', serialize)
    publisher = node.create_publisher(
        OccupancyGrid, '/map', QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    executable = Path(get_package_prefix('sentry_localization')) / 'lib' / (
        'sentry_localization/map_autosaver')
    process = subprocess.Popen([
        str(executable), '--ros-args', '-p', 'periodic_enabled:=false',
        '-p', 'serialize_timeout_s:=0.5', '-p', f'save_dir:={tmp_path / "backup"}',
    ])
    client = node.create_client(SaveMap, '/map_autosaver/save_now')

    def wait(predicate, timeout=10):
        deadline = time.monotonic() + timeout
        while not predicate():
            assert process.poll() is None, 'saver exited'
            assert time.monotonic() < deadline, 'test deadline exceeded'
            rclpy.spin_once(node, timeout_sec=0.02)

    def publish(mutate=None):
        grid = OccupancyGrid()
        grid.header.frame_id = 'map'
        grid.header.stamp = node.get_clock().now().to_msg()
        grid.info.width, grid.info.height = 2, 2
        grid.info.resolution = 0.05
        grid.info.origin.orientation.w = 1.0
        grid.data = [0, 100, -1, 0]
        if mutate:
            mutate(grid)
        publisher.publish(grid)
        # Wait for DDS delivery to the separate saver process.
        until = time.monotonic() + 0.1
        wait(lambda: time.monotonic() >= until)
        return grid

    def request(name='snapshot', validated=True, minimum=None):
        minimum = minimum or node.get_clock().now().to_msg()
        publish()
        return client.call_async(SaveMap.Request(
            directory=str(tmp_path / name), validated=validated, minimum_stamp=minimum))

    try:
        wait(client.service_is_ready)
        wait(lambda: publisher.get_subscription_count() > 0)
        # Let the serialization client discover its server before the first request.
        ready = time.monotonic() + 0.3
        wait(lambda: time.monotonic() >= ready)
        yield node, behavior, request, wait, publish
    finally:
        process.terminate()
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=5)
        node.destroy_service(service)
        node.destroy_node()
        rclpy.shutdown()
        if previous is None:
            os.environ.pop('ROS_DOMAIN_ID', None)
        else:
            os.environ['ROS_DOMAIN_ID'] = previous


@pytest.mark.parametrize('validated', [False, True])
def test_valid_bundle_and_no_overwrite(saver, tmp_path, validated):
    """Publish both representations and caller verdict, and preserve an existing bundle."""
    _, behavior, request, wait, _ = saver
    future = request(validated=validated)
    wait(future.done)
    assert future.result().success, future.result().message
    target = tmp_path / 'snapshot'
    for suffix in ('.posegraph', '.data', '.yaml', '.pgm'):
        assert (target / ('map' + suffix)).stat().st_size > 0
    assert json.loads((target / 'save.json').read_text()) == {'validated': validated}
    assert (target / 'map.pgm').read_bytes() == b'P5\n2 2\n255\n\xcd\xfe\xfe\x00'
    future = request()
    wait(future.done)
    assert not future.result().success
    assert behavior['calls'] == 1
    assert not list(tmp_path.glob('.mapping-*'))
    assert not (tmp_path / 'backup').exists()


@pytest.mark.parametrize('result,files', [(1, True), (0, False)])
def test_backend_failure_never_publishes(saver, tmp_path, result, files):
    """Service success without graph files cannot create a successful save marker."""
    _, behavior, request, wait, _ = saver
    behavior.update(result=result, files=files)
    future = request()
    wait(future.done)
    assert not future.result().success
    assert not (tmp_path / 'snapshot').exists()
    assert not list(tmp_path.glob('.mapping-*'))


def test_reset_stamp_rejects_cached_map(saver, tmp_path):
    """A caller's reset boundary excludes even a recently published pre-reset grid."""
    node, behavior, _, wait, publish = saver
    publish()
    minimum = node.get_clock().now().to_msg()
    client = node.create_client(SaveMap, '/map_autosaver/save_now')
    wait(client.service_is_ready)
    future = client.call_async(SaveMap.Request(
        directory=str(tmp_path / 'snapshot'), validated=True, minimum_stamp=minimum))
    wait(future.done)
    assert not future.result().success
    assert behavior['calls'] == 0
    assert not (tmp_path / 'snapshot').exists()


def test_timeout_and_busy(saver, tmp_path):
    """Timeout responds promptly, retains exclusion, and cleans up on a late reply."""
    node, behavior, request, wait, _ = saver
    behavior['delay'] = 1.0
    client = node.create_client(SaveMap, '/map_autosaver/save_now')
    wait(client.service_is_ready)
    minimum = node.get_clock().now().to_msg()
    first = request(minimum=minimum)
    # Dispatch a competing request before spinning the blocking fake backend.
    second = client.call_async(SaveMap.Request(
        directory=str(tmp_path / 'other'), minimum_stamp=minimum))
    wait(first.done)
    wait(second.done)
    assert not first.result().success
    assert 'timed out' in first.result().message
    assert not second.result().success
    assert 'busy' in second.result().message
    wait(lambda: not list(tmp_path.glob('.mapping-*')))
    assert not (tmp_path / 'snapshot').exists()
    assert not (tmp_path / 'other').exists()
    behavior['delay'] = 0.0
    recovered = request('recovered')
    wait(recovered.done)
    assert recovered.result().success, recovered.result().message


@pytest.mark.parametrize('invalid', ['frame', 'empty', 'unknown', 'resolution', 'future'])
def test_invalid_occupancy_never_serializes(saver, tmp_path, invalid):
    """Reject invalid occupancy content before asking the backend to write files."""
    node, behavior, _, wait, publish = saver
    client = node.create_client(SaveMap, '/map_autosaver/save_now')
    wait(client.service_is_ready)
    minimum = node.get_clock().now().to_msg()

    def mutate(grid):
        if invalid == 'frame':
            grid.header.frame_id = 'odom'
        elif invalid == 'empty':
            grid.info.width = 0
        elif invalid == 'unknown':
            grid.data = [-1] * 4
        elif invalid == 'resolution':
            grid.info.resolution = float('nan')
        elif invalid == 'future':
            grid.header.stamp.sec += 100

    publish(mutate)
    future = client.call_async(SaveMap.Request(
        directory=str(tmp_path / 'snapshot'), minimum_stamp=minimum))
    wait(future.done)
    assert not future.result().success
    assert behavior['calls'] == 0
    assert not (tmp_path / 'snapshot').exists()


def test_destination_created_during_serialization(saver, tmp_path):
    """Atomic publication cannot replace even a newly created empty directory."""
    _, behavior, request, wait, _ = saver
    behavior['collision'] = True
    future = request()
    wait(future.done)
    assert not future.result().success
    assert list((tmp_path / 'snapshot').iterdir()) == []
    assert not list(tmp_path.glob('.mapping-*'))
