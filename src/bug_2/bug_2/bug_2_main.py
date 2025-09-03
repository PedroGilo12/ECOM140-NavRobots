import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

import argparse

class Bug2Subscriber(Node):

    def __init__(self):
        super().__init__('Bug2Subscriber')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback,
            10
        )
        self.subscription

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.range_max)


def main(args=None):
    parser = argparse.ArgumentParser(description='Bug2Subscriber Node')
    parser.add_argument('--threshold', type=float, default=10.0, help='Threshold value')
    args, unknown = parser.parse_known_args()

    rclpy.init(args=unknown)

    print(args.threshold)

    minimal_subscriber = Bug2Subscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()