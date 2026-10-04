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

import os
import time

from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sentry_localization.map_image import to_pgm, to_yaml, write_atomic
from slam_toolbox.srv import SerializePoseGraph


class MapAutosaver(Node):
    """
    Save slam_toolbox's map every period_s into <save_dir>/<boot time>/.

    map.posegraph/.data via serialize_map (loadable with load_map:=true), and
    map.pgm/.yaml written here from /map. Not slam_toolbox's save_map: it
    starts a map_saver_cli process that must find /map within 2 s, and on the
    sentry often didn't. Each save overwrites the last, so a power cut loses
    at most period_s. A serialize still running skips its next turn.
    """

    def __init__(self):
        super().__init__('map_autosaver')
        self.declare_parameter('save_dir', '/workspaces/isaac_ros-dev/maps')
        self.declare_parameter('period_s', 30.0)
        run_dir = os.path.join(self.get_parameter('save_dir').value,
                               time.strftime('%Y%m%d-%H%M%S'))
        os.makedirs(run_dir, exist_ok=True)
        self._base = os.path.join(run_dir, 'map')
        self._serialize = self.create_client(SerializePoseGraph, '/slam_toolbox/serialize_map')
        self._serialize_pending = False
        self._map = None
        self._map_saved = None
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, '/map', self._on_map, qos)
        self.create_timer(self.get_parameter('period_s').value, self._tick)
        self.get_logger().info(f'saving the map to {self._base}.*')

    def _on_map(self, msg):
        self._map = msg

    def _tick(self):
        self._save_image()
        self._serialize_graph()

    def _save_image(self):
        m = self._map
        if m is None or m is self._map_saved:
            return
        started = time.monotonic()
        info, origin = m.info, m.info.origin
        try:
            write_atomic(self._base + '.pgm', to_pgm(m.data, info.width, info.height))
            write_atomic(self._base + '.yaml', to_yaml(
                'map.pgm', info.resolution, origin.position.x, origin.position.y,
                origin.orientation.z, origin.orientation.w))
        except (OSError, ValueError) as e:
            self.get_logger().warn(f'map.pgm failed: {e}')
            return
        self._map_saved = m
        self.get_logger().info(
            f'map.pgm ok ({info.width}x{info.height}) in {time.monotonic() - started:.2f} s')

    def _serialize_graph(self):
        if self._serialize_pending or not self._serialize.service_is_ready():
            return
        started = time.monotonic()
        future = self._serialize.call_async(SerializePoseGraph.Request(filename=self._base))
        self._serialize_pending = True

        def done(f):
            self._serialize_pending = False
            took = time.monotonic() - started
            result = f.result().result if f.result() is not None else None
            if result == 0:
                self.get_logger().info(f'serialize_map ok in {took:.1f} s')
            else:
                self.get_logger().warn(f'serialize_map failed ({result}) after {took:.1f} s')
        future.add_done_callback(done)


def main(args=None):
    try:
        rclpy.init(args=args)
        rclpy.spin(MapAutosaver())
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
