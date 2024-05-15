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
        self.declare_parameter('h_min', 5)
        self.declare_parameter('h_max', 15)
        self.declare_parameter('s_min', 180)
        self.declare_parameter('s_max', 255)
        self.declare_parameter('v_min', 140)
        self.declare_parameter('v_max', 255)
        # Declare the contour parameters with default values
        self.declare_parameter('area_size', 800)

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
        area_size = self.get_parameter('area_size').value

        # Change the hsv paramaters
        h_min = self.get_parameter('h_min').value
        h_max = self.get_parameter('h_max').value
        s_min = self.get_parameter('s_min').value
        s_max = self.get_parameter('s_max').value
        v_min = self.get_parameter('v_min').value
        v_max = self.get_parameter('v_max').value
        # Change the contour area parameters
        area_size = self.get_parameter('area_size').value

        # Mask the original BGR image with the edge-detected image

        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        lower_orange = np.array([h_min, s_min, v_min])
        upper_orange = np.array([h_max, s_max, v_max])
        mask = cv2.inRange(hsv, lower_orange, upper_orange)

        erosion = cv2.erode(mask, None, iterations=2)
        dilation = cv2.dilate(erosion, None, iterations=2)  

        contours, _ = cv2.findContours(dilation, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        draw = cv_image.copy()
        midpoint = None
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > area_size:
                center, radius = cv2.minEnclosingCircle(contour)
                center = (int(center[0]), int(center[1]))
                radius = int(radius)
                if radius > 10:
                    cv2.circle(draw, center, radius, (0, 255, 0), 2)
                    coord_text = f"({center[0]}, {center[1]})"
                    cv2.putText(draw, coord_text, (center[0] - radius, center[1] - radius - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
                    
                    midpoint = Point(x=float(center[0]), y=float(center[1]), z=float(radius))
                    self.position_publisher.publish(midpoint)
                    #self.get_logger().info(f"Ball position: X: {center[0]}, Y: {center[1]}, Radius: {radius}")

        if len (contours) == 0:
            self.get_logger().info("NO BAAAALLLLLSSSSS!!!!!!")
        try:
            output_image = self.bridge.cv2_to_imgmsg(draw, "bgr8")
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
