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

import math

from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from sentry_localization.map_pose_core import compose, rotate_cov_xy
from tf2_ros import Buffer, TransformException, TransformListener

# xy variance (m^2) published until the backend's first pose arrives, so
# mcb_relay's max_std_m gate refuses to relocalize from it.
UNKNOWN_VAR = 1.0


def _yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class MapPosePublisher(Node):
    """
    Publish /localization/odom moved into the map frame, on /localization/map_odom.

    Pose: map->odom (TF, at the input's stamp, else the latest) composed with
    the input's odom->root. Stamp and twist (child frame) are the input's.
    xy covariance: the input's, rotated into map, plus the latest xy
    covariance on backend_pose_topic (/amcl_pose or slam_toolbox's /pose).
    Not launched under localization_mode:=none, which has no map frame.
    """

    def __init__(self):
        super().__init__('map_pose_publisher')
        self.declare_parameter('input_topic', '/localization/odom')
        self.declare_parameter('output_topic', '/localization/map_odom')
        self.declare_parameter('backend_pose_topic', '/amcl_pose')
        self.declare_parameter('map_frame', 'map')
        self._map_frame = self.get_parameter('map_frame').value
        self._backend_cov = None
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._pub = self.create_publisher(
            Odometry, self.get_parameter('output_topic').value, 10)
        self.create_subscription(
            PoseWithCovarianceStamped, self.get_parameter('backend_pose_topic').value,
            self._backend_cb, 10)
        self.create_subscription(
            Odometry, self.get_parameter('input_topic').value, self._odom_cb, 10)

    def _backend_cb(self, msg):
        c = msg.pose.covariance
        self._backend_cov = ((c[0], c[1]), (c[6], c[7]))

    def _map_odom(self, odom_frame, stamp):
        for t in (Time.from_msg(stamp), Time()):
            try:
                tf = self._tf_buffer.lookup_transform(self._map_frame, odom_frame, t)
            except TransformException:
                continue
            tr = tf.transform.translation
            return tr.x, tr.y, _yaw(tf.transform.rotation)
        return None

    def _odom_cb(self, msg):
        map_odom = self._map_odom(msg.header.frame_id, msg.header.stamp)
        if map_odom is None:
            self.get_logger().warn(
                f'no {self._map_frame}->{msg.header.frame_id} yet; not publishing',
                throttle_duration_sec=5.0)
            return
        p = msg.pose.pose
        x, y, yaw = compose(map_odom, (p.position.x, p.position.y, _yaw(p.orientation)))
        c = msg.pose.covariance
        (xx, xy), (_, yy) = rotate_cov_xy(((c[0], c[1]), (c[6], c[7])), map_odom[2])
        (bxx, bxy), (_, byy) = self._backend_cov or ((UNKNOWN_VAR, 0.0), (0.0, UNKNOWN_VAR))

        out = Odometry()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self._map_frame
        out.child_frame_id = msg.child_frame_id
        out.pose.pose.position.x, out.pose.pose.position.y = x, y
        out.pose.pose.position.z = p.position.z
        out.pose.pose.orientation.z = math.sin(yaw / 2.0)
        out.pose.pose.orientation.w = math.cos(yaw / 2.0)
        cov = list(c)
        cov[0], cov[1], cov[6], cov[7] = xx + bxx, xy + bxy, xy + bxy, yy + byy
        out.pose.covariance = cov
        out.twist = msg.twist
        self._pub.publish(out)


def main(args=None):
    try:
        rclpy.init(args=args)
        rclpy.spin(MapPosePublisher())
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
