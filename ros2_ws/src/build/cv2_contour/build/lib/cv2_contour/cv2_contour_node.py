import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2
import numpy as np

class CV2ContourNode(Node):
    def __init__(self):
        super().__init__('cv2_contour_node')
        # Declare the hsv parameters with default values
        self.declare_parameter('h_min', 8)
        self.declare_parameter('h_max', 50)
        self.declare_parameter('s_min', 40)
        self.declare_parameter('s_max', 255)
        self.declare_parameter('v_min', 190)
        self.declare_parameter('v_max', 255)
        # Declare the contour parameters with default values
        self.declare_parameter('contour_area_min', 300)
        self.declare_parameter('contour_area_max', 950)

        self.subscription = self.create_subscription(
            Image,
            'blurred_image',
            self.image_callback,
            10)

        self.image_publisher = self.create_publisher(
            Image,
            'output_ball',
            10)
        
        self.position_publisher = self.create_publisher(
            Point, 'ball_position', 10)
        
        self.mask_publisher = self.create_publisher(
            Image,
            'hsv_mask',
            10)
        
        self.bridge = CvBridge()

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error('Failed to convert image: %s' % str(e))
            return
        # Change the hsv paramaters
        h_min = self.get_parameter('h_min').value
        h_max = self.get_parameter('h_max').value
        s_min = self.get_parameter('s_min').value
        s_max = self.get_parameter('s_max').value
        v_min = self.get_parameter('v_min').value
        v_max = self.get_parameter('v_max').value
        # Change the contour parameters
        contour_area_min = self.get_parameter('contour_area_min').value
        contour_area_max = self.get_parameter('contour_area_max').value

        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        lower_orange = np.array([h_min, s_min, v_min])
        upper_orange = np.array([h_max, s_max, v_max])
        mask = cv2.inRange(hsv, lower_orange, upper_orange)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        midpoint = None
        for contour in contours:
            if contour_area_min <= cv2.contourArea(contour) <= contour_area_max:
                x, y, w, h = cv2.boundingRect(contour)
                midpoint = Point(x=x+w/2, y=y+h/2, z=0.0)
                cv2.rectangle(cv_image, (x, y), (x+w, y+h), (0, 255, 0), 2)
                coord_text = f"({int(midpoint.x)}, {int(midpoint.y)})"
                cv2.putText(cv_image, coord_text, (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)


        try:
            output_image = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
            mask_image = self.bridge.cv2_to_imgmsg(mask, "mono8")
            self.mask_publisher.publish(mask_image)
            self.image_publisher.publish(output_image)
            if midpoint is not None:
                self.position_publisher.publish(midpoint)
        except Exception as e:
            self.get_logger().error('Failed to convert and publish processed image: %s' % str(e))

def main(args=None):
    rclpy.init(args=args)
    node = CV2ContourNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
