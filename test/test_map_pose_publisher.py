# Copyright 2026 Thornbots
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Verify stamped composition, one result per odometry input and the lockstep backend wait."""
import math
from pathlib import Path
import signal
import subprocess
import time

from ament_index_python.packages import get_package_prefix
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import rclpy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header
from std_srvs.srv import Empty


def test_stamps_freshness_and_missing_transform():
    rclpy.init()
    node = rclpy.create_node('map_pose_contract_test')
    outputs = []
    refreshes = []

    def refresh(request, response):
        refreshes.append(request)
        return response

    node.create_service(Empty, '/test/nomotion_update', refresh)
    subscription = node.create_subscription(Odometry, '/test/map_result', outputs.append, 100)
    odom = node.create_publisher(Odometry, '/test/local_pose', 10)
    backend = node.create_publisher(PoseWithCovarianceStamped, '/test/backend_pose', 10)
    executable = Path(get_package_prefix('sentry_localization')) / (
        'lib/sentry_localization/map_pose_publisher')
    process = subprocess.Popen([str(executable), '--ros-args',
                                '-p', 'input_topic:=/test/local_pose',
                                '-p', 'output_topic:=/test/map_result',
                                '-p', 'backend_pose_topic:=/test/backend_pose',
                                '-p', 'map_frame:=test_map',
                                '-p', 'nomotion_update_service:=/test/nomotion_update'])

    def wait(predicate):
        end = time.monotonic() + 10
        while not predicate():
            assert process.poll() is None
            assert time.monotonic() < end
            rclpy.spin_once(node, timeout_sec=.01)

    def flush():
        end = time.monotonic() + .15
        wait(lambda: time.monotonic() >= end)

    def sample(seconds, frame='test_nav'):
        message = Odometry()
        message.header.stamp = Time(sec=int(seconds), nanosec=int((seconds % 1)*1e9))
        message.header.frame_id, message.child_frame_id = frame, 'root'
        message.pose.pose.position.x = 2.
        message.pose.pose.orientation.w = 1.
        count = len(outputs)
        odom.publish(message)
        wait(lambda: len(outputs) > count)
        flush()
        assert len(outputs) == count + 1
        result = outputs[-1]
        assert result.header.stamp == message.header.stamp
        assert result.header.frame_id == 'test_map'
        return result

    try:
        wait(lambda: odom.get_subscription_count() and backend.get_subscription_count()
             and subscription.get_publisher_count())
        flush()
        # Missing backend still produces an invalid result for the relay handshake.
        assert math.isnan(sample(1.).pose.pose.position.x)
        assert math.isnan(sample(2.).pose.pose.position.x)
        # Continue ticks while service discovery completes, as a parked EKF does.
        for _ in range(20):
            if refreshes:
                break
            sample(2.)
        assert refreshes  # recover a missed initial backend publication
        pose = PoseWithCovarianceStamped()
        pose.header.stamp, pose.header.frame_id = Time(sec=1, nanosec=500000000), 'test_map'
        pose.pose.pose.position.x, pose.pose.pose.position.y = 1., 2.
        pose.pose.pose.orientation.z, pose.pose.pose.orientation.w = math.sqrt(.5), math.sqrt(.5)
        pose.pose.covariance[0], pose.pose.covariance[7], pose.pose.covariance[35] = .04, .09, .01
        backend.publish(pose)
        flush()
        # No global TF has been published: derive correction at backend capture time.
        result = sample(2.1)
        assert abs(result.pose.pose.position.x - 1.) < 1e-9
        assert abs(result.pose.pose.position.y - 2.) < 1e-9
        assert abs(result.pose.covariance[0] - .04) < 1e-9
        assert math.isnan(sample(3.).pose.pose.position.x)  # stale backend covariance
        pose.header.stamp = Time(sec=5)
        backend.publish(pose)
        flush()
        assert math.isnan(sample(4.5).pose.pose.position.x)  # future covariance
        assert math.isnan(sample(.5).pose.pose.position.x)  # clock reset
    finally:
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=5)
        node.destroy_node()
        rclpy.shutdown()


def test_lockstep_backend_results_wait_for_answered_scans():
    rclpy.init()
    node = rclpy.create_node('map_pose_lockstep_test')
    outputs, seen, refreshes = [], [], []
    node.create_service(
        Empty, '/test/nomotion_update', lambda req, resp: refreshes.append(req) or resp)
    subscription = node.create_subscription(Odometry, '/test/map_result', outputs.append, 100)
    node.create_subscription(Header, '/test/scan_seen', seen.append, 100)
    odom = node.create_publisher(Odometry, '/test/local_pose', 10)
    results = node.create_publisher(PoseWithCovarianceStamped, '/test/scan_result', 100)
    scans = node.create_publisher(LaserScan, '/test/scan', 10)
    executable = Path(get_package_prefix('sentry_localization')) / (
        'lib/sentry_localization/map_pose_publisher')
    process = subprocess.Popen([str(executable), '--ros-args',
                                '-p', 'input_topic:=/test/local_pose',
                                '-p', 'output_topic:=/test/map_result',
                                '-p', 'map_frame:=test_map',
                                '-p', 'nomotion_update_service:=/test/nomotion_update',
                                '-p', 'backend_result_topic:=/test/scan_result',
                                '-p', 'backend_delay_s:=0.05',
                                '-p', 'scan_topic:=/test/scan',
                                '-p', 'scan_seen_topic:=/test/scan_seen'])

    def wait(predicate):
        end = time.monotonic() + 10
        while not predicate():
            assert process.poll() is None
            assert time.monotonic() < end
            rclpy.spin_once(node, timeout_sec=.01)

    def flush():
        end = time.monotonic() + .15
        wait(lambda: time.monotonic() >= end)

    def stamp(seconds):
        return Time(sec=int(seconds), nanosec=int(round((seconds % 1)*1e9)))

    def tick(seconds):
        message = Odometry()
        message.header.stamp = stamp(seconds)
        message.header.frame_id, message.child_frame_id = 'test_nav', 'root'
        message.pose.pose.orientation.w = 1.
        odom.publish(message)

    def result(seconds, x=None):
        message = PoseWithCovarianceStamped()
        message.header.stamp = stamp(seconds)
        if x is not None:
            message.header.frame_id = 'test_map'
            message.pose.pose.position.x = x
            message.pose.pose.orientation.w = 1.
            message.pose.covariance[0] = message.pose.covariance[7] = .01
            message.pose.covariance[35] = .01
        results.publish(message)
        flush()

    def scan(seconds):
        message = LaserScan()
        message.header.stamp, message.header.frame_id = stamp(seconds), 'lidar'
        count = len(seen)
        scans.publish(message)
        wait(lambda: len(seen) > count)
        assert seen[-1].stamp == message.header.stamp

    try:
        wait(lambda: odom.get_subscription_count() and results.get_subscription_count()
             and scans.get_subscription_count() and subscription.get_publisher_count())
        flush()
        result(.9)  # the backend is live: an answer with no pose
        scan(1.)
        tick(1.)
        tick(1.04)  # scan 1.0 is not due until 1.05
        wait(lambda: len(outputs) == 2)
        assert all(math.isnan(o.pose.pose.position.x) for o in outputs)
        tick(1.06)
        flush()
        assert len(outputs) == 2  # waits for the backend's answer to scan 1.0
        result(1., x=3.)
        wait(lambda: len(outputs) == 3)
        assert outputs[2].header.stamp == stamp(1.06)
        assert abs(outputs[2].pose.pose.position.x - 3.) < 1e-9
        scan(1.03)
        result(1.03, x=7.)  # answered early, but not due until 1.08
        tick(1.07)
        wait(lambda: len(outputs) == 4)
        assert abs(outputs[3].pose.pose.position.x - 3.) < 1e-9
        tick(1.08)
        wait(lambda: len(outputs) == 5)
        assert abs(outputs[4].pose.pose.position.x - 7.) < 1e-9
        assert not refreshes  # the backend refreshes itself in this mode
    finally:
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=5)
        node.destroy_node()
        rclpy.shutdown()
