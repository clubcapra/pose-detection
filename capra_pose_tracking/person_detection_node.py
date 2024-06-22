import sys
import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped

from .pose_detection.tools.pose_detector import PoseDetector
import pyzed.sl as sl
from zed_interfaces.msg import ObjectsStamped

class TrackingNode(Node):

    def __init__(self):
        super().__init__('tracking_node')
        # zed subscription
        self.subscription = self.create_subscription(
            ObjectsStamped,
            '/zed2i/zed_node/body_trk/skeletons',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        
        # position/state publisher
        # self.publisherPoint = self.create_publisher(PointStamped, 'tracking/point', 10)
        # self.publisherState = self.create_publisher(PointStamped, 'tracking/state', 10)
        
    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg)
        # self.publish_data(self.publisherPoint, msg)

    def publish_data(self, publisher: Publisher, data):
        publisher.publish(data)
        self.get_logger().info('Publishing: "%s"' % data)

def main(args=None):
    rclpy.init(args=args)

    trackingNode = TrackingNode()

    rclpy.spin(trackingNode)


if __name__ == '__main__':
    main()