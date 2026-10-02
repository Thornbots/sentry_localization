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

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from slam_toolbox.srv import SaveMap, SerializePoseGraph


class MapAutosaver(Node):
    """
    Save slam_toolbox's map every period_s into <save_dir>/<boot time>/.

    map.posegraph/.data (serialize_map, loadable with load_map:=true) and
    map.pgm/.yaml (save_map), alternating every period_s / 2: run together,
    save_map timed out on /map. Each save overwrites the last, so a power
    cut loses at most period_s. A call still running skips its next turn.
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
        self._save = self.create_client(SaveMap, '/slam_toolbox/save_map')
        self._pending = {}
        self._turn = 0
        self.create_timer(self.get_parameter('period_s').value / 2.0, self._tick)
        self.get_logger().info(f'saving the map to {self._base}.*')

    def _tick(self):
        self._turn ^= 1
        if self._turn:
            self._call('serialize_map', self._serialize,
                       SerializePoseGraph.Request(filename=self._base))
            return
        req = SaveMap.Request()
        req.name.data = self._base
        self._call('save_map', self._save, req)

    def _call(self, name, client, request):
        if name in self._pending or not client.service_is_ready():
            return
        started = time.monotonic()
        future = client.call_async(request)
        self._pending[name] = future

        def done(f):
            del self._pending[name]
            took = time.monotonic() - started
            result = f.result().result if f.result() is not None else None
            if result == 0:
                self.get_logger().info(f'{name} ok in {took:.1f} s')
            else:
                self.get_logger().warn(f'{name} failed ({result}) after {took:.1f} s')
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
