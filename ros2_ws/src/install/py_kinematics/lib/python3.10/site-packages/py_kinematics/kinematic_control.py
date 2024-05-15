import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Float64MultiArray
import numpy as np
from . import kinematic_functions as kin  # Ensure this import works, adjust the path as needed

class KinematicController(Node):
    def __init__(self):
        super().__init__('kinematic_controller')
        # Subscribers for control efforts
        self.subscription_x = self.create_subscription(
            Float64,
            'control_effort_x',
            self.listener_callback_x,
            10)
        self.subscription_y = self.create_subscription(
            Float64,
            'control_effort_y',
            self.listener_callback_y,
            10)
        
        # Publisher for motor outputs
        self.publisher_motors = self.create_publisher(Float64MultiArray, 'motor_outputs', 10)

        # Initial control efforts
        self.control_effort_x = 0.0
        self.control_effort_y = 0.0

        # Timer setup to repeatedly call timer_callback method
        self.timer = self.create_timer(1.0, self.timer_callback)  # Adjusted to 10 Hz

    def listener_callback_x(self, msg):
        self.control_effort_x = msg.data
        self.get_logger().info(f'X control effort received: {msg.data}')
        self.perform_kinematic_calculations()

    def listener_callback_y(self, msg):
        self.control_effort_y = msg.data
        self.get_logger().info(f'Y control effort received: {msg.data}')
        self.perform_kinematic_calculations()

    def perform_kinematic_calculations(self):
        """Calculate kinematics and publish motor commands."""
        # Simulate kinematic calculation
        matrix = kin.updateMatrix(self.control_effort_x, self.control_effort_y, 25)
        servoPos = kin.setServos(matrix, 4)
        motor_a = np.rad2deg(servoPos[0])
        motor_b = np.rad2deg(servoPos[1])
        motor_c = np.rad2deg(servoPos[2])

        # Create and publish the message
        motor_msg = Float64MultiArray()
       # rounded_data = [
        #    np.round(motor_a, 2),
         #   np.round(motor_b, 2),
          #  np.round(motor_c, 2)]
        rounded_data = [60.0, 60.0, 60.0]
        motor_msg.data = rounded_data
        self.publisher_motors.publish(motor_msg)
        self.get_logger().info(f'Published motor commands: {motor_msg.data}')

    def timer_callback(self):
        """Periodic checking of system status or other functions."""
        self.get_logger().info('Timer callback triggered')

def main(args=None):
    rclpy.init(args=args)
    kinematic_controller = KinematicController()
    rclpy.spin(kinematic_controller)
    kinematic_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
