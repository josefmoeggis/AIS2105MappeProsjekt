import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import serial
import math

class MotorControllerNode(Node):

    def __init__(self):
        super().__init__('motor_controller_node')
        
        # Replace '/dev/ttyUSB0' with the appropriate serial port for your Arduino
        self.serial_port = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
        
        # Subscription to the 'motor_outputs' topic
        self.subscription = self.create_subscription(
            Float64MultiArray,
            'motor_outputs',
            self.listener_callback,
            10
        )
          

    def listener_callback(self, msg):
        if len(msg.data) == 3:
            # Extract the servo positions
            m1 = msg.data[0]
            m2 = msg.data[1]
            m3 = msg.data[2]

            self.get_logger().info(f'Motor commands received: m1: {m1} m2: {m2} m3: {m3}')
            # Create the command string
            command = f"MOTORS:{m1},{m2},{m3}\n"
            self.get_logger().info(f'Sending command: {command}')
            
            # Write the command to the serial port
            self.serial_port.write(command.encode('utf-8'))
        else:
            self.get_logger().warn('Received invalid data length')

def main(args=None):

    rclpy.init(args=args)
    kinematic_controller = MotorControllerNode()
    rclpy.spin(kinematic_controller)
    kinematic_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
