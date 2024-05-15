import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import yaml
import numpy as np

# Load camera calibration parameters from the calibrated YAML file.
def load_camera_calibration(filename):


    with open(filename, "r") as file:
        calibration_data = yaml.safe_load(file)

    image_width = calibration_data["image_width"]
    image_height = calibration_data["image_height"]
    camera_matrix = np.array(calibration_data["camera_matrix"]["data"]).reshape(3, 3)
    distortion_coefficients = np.array(calibration_data["distortion_coefficients"]["data"])
    rectification_matrix = np.array(calibration_data["rectification_matrix"]["data"]).reshape(3, 3)
    projection_matrix = np.array(calibration_data["projection_matrix"]["data"]).reshape(3, 4)

    return (image_width, image_height, camera_matrix, distortion_coefficients, rectification_matrix, projection_matrix)

def undistort_image(image, camera_matrix, distortion_coefficients, new_camera_matrix=None):
    # Undistort the input image using the camera matrix and distortion coefficients.
    h, w = image.shape[:2]
    if new_camera_matrix is None:
        new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(camera_matrix, distortion_coefficients, (w, h), 1, (w, h))
    else:
        _, roi = cv2.getOptimalNewCameraMatrix(camera_matrix, distortion_coefficients, (w, h), 1, (w, h))
    undistorted_image = cv2.undistort(image, camera_matrix, distortion_coefficients, None, new_camera_matrix)
    x, y, w, h = roi
    undistorted_image = undistorted_image[y:y+h, x:x+w]
    return undistorted_image

class UndistortImageNode(Node):
    def __init__(self):
        super().__init__('undistort_image_node')

        # Load calibration parameters
        
        calibration_file = '/home/joe/Documents/ost.yaml'
        (self.image_width, self.image_height, self.camera_matrix,
         self.distortion_coefficients, self.rectification_matrix,
         self.projection_matrix) = load_camera_calibration(calibration_file)

        self.subscription = self.create_subscription(
            Image,
            'image_raw',
            self.image_callback,
            10)

        self.publisher = self.create_publisher(
            Image,
            'undistorted_image',
            10)

        self.bridge = CvBridge()

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f'cv_bridge exception: {e}')
            return

        # Undistort the image
        undistorted_image = undistort_image(cv_image, self.camera_matrix, self.distortion_coefficients)

        try:
            undistorted_msg = self.bridge.cv2_to_imgmsg(undistorted_image, 'bgr8')
            self.publisher.publish(undistorted_msg)
        except CvBridgeError as e:
            self.get_logger().error(f'Failed to convert and publish undistorted image: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = UndistortImageNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
