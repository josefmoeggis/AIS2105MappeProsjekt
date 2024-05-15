import rclpy
from rclpy.node import Node # Import the custom message type
from std_msgs.msg import Float64MultiArray
import numpy as np
from . import kinematic_functions as kin  # Ensure this import works, adjust the path as needed

class KinematicController(Node):
    def __init__(self):
        super().__init__('kinematic_controller')
        
        # Subscription to the 'control_efforts' topic
        self.subscription = self.create_subscription(
            Float64MultiArray,
            'control_efforts',
            self.listener_callback,
            10
        )
        
        # Publisher for motor outputs
        self.publisher_motors = self.create_publisher(Float64MultiArray, 'motor_outputs', 10)

        # Initial control efforts
        self.control_effort_x = 0.0
        self.control_effort_y = 0.0

        # Timer setup to repeatedly call timer_callback method
        self.timer = self.create_timer(0.1, self.timer_callback)  # Adjusted to 1 Hz for debugging

    def listener_callback(self, msg):
        self.control_effort_x = msg.data[0]
        self.control_effort_y = msg.data[1]
        self.get_logger().info(f'Control efforts received: x={msg.data[0]}, y={msg.data[1]}')
        self.perform_kinematic_calculations()

    def perform_kinematic_calculations(self):
        """Calculate kinematics and publish motor commands."""
        self.get_logger().info('Performing kinematic calculations.')

        # Simulate kinematic calculation
        matrix = kin.updateMatrix(self.control_effort_x, self.control_effort_y, 25)
        self.get_logger().info(f'Matrix: {matrix}')
        matrix[2][0] += 3
        matrix[2][1] += 3
        matrix[2][2] += 3
        servoPos = kin.setServos(matrix, 4)
        self.get_logger().info(f'Servo Positions: {servoPos}')
        '''
        if np.any(np.isnan(servoPos)):
            self.get_logger().error('NaN detected in servo positions')
            # Force the output to [0, 0, 0] if NaN detected
            rounded_data = [0.0, 0.0, 0.0]
        else:'''

        motor_a = np.rad2deg(servoPos[0]) 
        motor_b = np.rad2deg(servoPos[1]) 
        motor_c = np.rad2deg(servoPos[2]) 
        rounded_data = [motor_a, motor_b, motor_c]

        # Create and publish the message
        motor_msg = Float64MultiArray()
        motor_msg.data = rounded_data
        self.publisher_motors.publish(motor_msg)
        self.get_logger().info(f'Published motor commands: {rounded_data}')

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
