import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class PassthroughOdomPublisher(Node):
    """
    Relays /odom onto /localization/odom unchanged.

    Used for localization_mode in {slam, mapping, amcl}, where root pose
    is uncorrected wheel odometry (only map->odom is corrected, by
    slam_toolbox/amcl directly). Not launched for localization_mode:=ekf,
    which publishes /localization/odom itself instead.
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
