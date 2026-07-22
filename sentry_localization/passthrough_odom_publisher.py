import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class PassthroughOdomPublisher(Node):
    """
    Relays /odom onto /localization/odom unchanged.

    Used for localization_mode in {slam, mapping, amcl}: root pose in
    these modes is uncorrected wheel odometry (only map->odom is
    corrected, via slam_toolbox/amcl scan-matching, which those nodes
    broadcast directly). sentry_pkg's odom_tf_broadcaster always reads
    /localization/odom regardless of backend, so this relay exists purely
    to give it something to subscribe to in the modes that don't
    otherwise correct odom->root -- localization_mode:=ekf publishes
    /localization/odom itself instead (ekf_node's remapped output), so
    this node isn't launched there.
    """

    def __init__(self):
        super().__init__('passthrough_odom_publisher')
        self._pub = self.create_publisher(Odometry, '/localization/odom', 10)
        self.create_subscription(Odometry, '/odom', self._odom_cb, 10)

    def _odom_cb(self, msg):
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PassthroughOdomPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
