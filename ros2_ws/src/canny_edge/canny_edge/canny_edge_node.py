import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CannyEdgeNode(Node):
    def __init__(self):
        super().__init__('canny_edge_node')

        # Declare parameters with default values
        self.declare_parameter('lower_threshold', 100)
        self.declare_parameter('upper_threshold', 200)
        # Subscription to the raw image topic
        self.subscription = self.create_subscription(
            Image,
            'image_raw',
            self.image_callback,
            10)

        # Publisher for the edge-detected image
        self.publisher = self.create_publisher(
            Image,
            'output_edges',
            10)
        self.bridge = CvBridge()

    def image_callback(self, msg):
        try:
            # Convert ROS image message to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error('Failed to convert image: %s' % str(e))
            return

        # Retrieve the current value of the thresholds
        lower_threshold = self.get_parameter('lower_threshold').value
        upper_threshold = self.get_parameter('upper_threshold').value

        grayscale_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        try:
            # Apply Canny Edge Detection with dynamic thresholds
            edges = cv2.Canny(grayscale_image, lower_threshold, upper_threshold)
        except Exception as e:
            self.get_logger().error('Failed to apply Canny Edge Detection: %s' % str(e))
            return

        try:
            # Convert the edge-detected image back to ROS format and publish
            edge_msg = self.bridge.cv2_to_imgmsg(edges, "mono8")
            self.publisher.publish(edge_msg)
        except Exception as e:
            self.get_logger().error('Failed to convert and publish edge-detected image: %s' % str(e))

def main(args=None):
    rclpy.init(args=args)
    canny_edge_node = CannyEdgeNode()
    rclpy.spin(canny_edge_node)
    canny_edge_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
