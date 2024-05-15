import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class GaussianBlurNode(Node):
    def __init__(self):
        super().__init__('gaussian_blur_node')

        # Declare and initialize the blur_radius parameter
        self.declare_parameter('blur_radius', 5)

        # Subscription to the raw image topic
        self.subscription = self.create_subscription(
            Image,
            'undistorted_image',
            self.image_callback,
            10)

        # Publisher for the blurred image
        self.publisher = self.create_publisher(
            Image,
            'blurred_image',
            10)
        self.bridge = CvBridge()

    def image_callback(self, msg):
        try:
            # Convert ROS image message to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error('Failed to convert image: %s' % str(e))
            return

        # Retrieve the current value of the blur_radius parameter
        blur_radius = self.get_parameter('blur_radius').value

        # Create kernel size from blur radius parameter
        kernel_size = (blur_radius, blur_radius)

        try:
            # Apply Gaussian blur using the dynamic kernel size
            blurred_image = cv2.GaussianBlur(cv_image, kernel_size, 0)
        except Exception as e:
            self.get_logger().error('Failed to apply Gaussian blur: %s' % str(e))
            return

        try:
            # Convert back to ROS image format and publish
            msg_blurred = self.bridge.cv2_to_imgmsg(blurred_image, 'bgr8')
            self.publisher.publish(msg_blurred)
        except Exception as e:
            self.get_logger().error('Failed to convert and publish blurred image: %s' % str(e))

def main(args=None):
    rclpy.init(args=args)
    gaussian_blur_node = GaussianBlurNode()
    rclpy.spin(gaussian_blur_node)
    gaussian_blur_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
