import rclpy
from rclpy.node import Node # Import the custom message type
from std_msgs.msg import Float64MultiArray
import numpy as np
from geometry_msgs.msg import Point
from . import magnus_functions as mf
import time

class MagnusKinematicNode(Node):
    def __init__(self):
        super().__init__('magnus_kinematic_node')
        
        self.subscription = self.create_subscription(
            Point,
            'ball_position',
            self.listener_callback,
            10
        )
        self.declare_parameter('kp', 0.5)
        self.declare_parameter('ki', 0.0)
        self.declare_parameter('kd', 0.1)

        # Publisher for motor outputs
        self._parameters
        self.publisher_motors = self.create_publisher(Float64MultiArray, 'motor_outputs', 10)

        self.publish_to_rviz = self.create_publisher(Float64MultiArray, 'angle_outputs', 10)

        self.target_x = 290
        self.target_y = 207
        
        self.prev_e_x = 0
        self.prev_e_y = 0
        self.xIntegral = 0
        self.yIntegral = 0

        self.last_time = time.time()
        self.dt = 0
    def listener_callback(self, msg):
        current_t = time.time()
        self.dt = current_t - self.last_time


        kp = self.get_parameter('kp').value
        ki = self.get_parameter('ki').value
        kd = self.get_parameter('kd').value

        if msg:
            
            # The callback has now found data in the topic
            pid_output_x, self.prev_e_x, self.xIntegral = mf.PID(self.target_x, msg.x, self.dt, self.prev_e_x, self.xIntegral, 5, kp, ki, kd)
            pid_output_y, self.prev_e_y, self.yIntegral = mf.PID(self.target_y, msg.y, self.dt, self.prev_e_y, self.yIntegral, 5, kp, ki, kd)

            pitch, roll = mf.coordinates2angle(pid_output_x, pid_output_y)
            angle_outputs = Float64MultiArray()
            angle_outputs.data = [roll, pitch]
            s1,s2,s3 = mf.adjust_platform(-pitch, -roll, 20)
            ss1 = (s1 + 90)
            ss2 = (s2 + 90)
            ss3 = (s3 + 90)
            # PRINT SERVO ANGLES
            myMotorOutputs = Float64MultiArray()
            myMotorOutputs.data = [ss1,ss2,ss3]
            self.publisher_motors.publish(myMotorOutputs)
            self.publish_to_rviz.publish(angle_outputs)
            self.get_logger().info(f'Published motor commands: {myMotorOutputs.data}')
        else:
            self.get_logger().info(f"There is no data in the topic!")

def main(args=None):
    rclpy.init(args=args)
    kinematic_controller = MagnusKinematicNode()
    rclpy.spin(kinematic_controller)
    kinematic_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
