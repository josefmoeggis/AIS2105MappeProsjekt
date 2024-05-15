import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import time
import numpy as np
from geometry_msgs.msg import Point

class BallPIDController(Node):
    def __init__(self):
        super().__init__('ball_pid_controller')
        # Publisher for control efforts
        self.publisher = self.create_publisher(Float64MultiArray, 'control_efforts', 10)
        
        # Timer to publish control efforts every second
        self.timer_period = 0.1  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        
        # Subscriber for actual ball positions
        self.subscription = self.create_subscription(
            Point,
            'ball_position',
            self.listener_callback,
            10
        )

        # PID parameters for X and Y
        self.setpoint_x = 0.0  # Desired position X
        self.setpoint_y = 0.0  # Desired position Y

        self.kp = 1.0
        self.ki = 0.0
        self.kd = 0.0

        self.integral_x = 0.0
        self.last_error_x = 0.0
        self.integral_y = 0.0
        self.last_error_y = 0.0

        self.process_value_x = 0.0
        self.process_value_y = 0.0

        self.last_time = time.time()  # Initialize last_time

    def map_range_x(self, x):
        return (x - 292) #* (17.75 - -17.75) / (478 - 104) + (-17.75)   
    
    def map_range_y(self, y):
        return (y - 220) #* (17.75 - -17.75) / (31.5 - 408.5) + (-17.75)
    
    def listener_callback(self, msg):
        self.process_value_x = self.map_range_x(msg.x)  # Map from pixels to cm based on plate size
        self.process_value_y = self.map_range_y(msg.y)

    def timer_callback(self):
        current_t = time.time()
        self.dt = current_t - self.last_time
        self.last_time = current_t

        # PID Control for X
        error_x = self.setpoint_x - self.process_value_x
        self.integral_x += error_x * self.timer_period
        derivative_x = (error_x - self.last_error_x) / self.timer_period
        control_effort_x = (self.kp * error_x) + (self.ki * self.integral_x) + (self.kd * derivative_x)

        # PID Control for Y
        error_y = self.setpoint_y - self.process_value_y
        self.integral_y += error_y * self.dt
        derivative_y = (error_y - self.last_error_y) / self.dt
        control_effort_y = (self.kp * error_y) + (self.ki * self.integral_y) + (self.kd * derivative_y)

        # Publish control efforts
        ctrl_msg = Float64MultiArray(data=[control_effort_x, control_effort_y])
        self.publisher.publish(ctrl_msg)

        # Log the published control efforts
        self.get_logger().info(f'Published control efforts: x={control_effort_x}, y={control_effort_y}')

        # Update last errors
        self.last_error_x = error_x
        self.last_error_y = error_y

def main(args=None):
    rclpy.init(args=args)
    ball_controller = BallPIDController()
    rclpy.spin(ball_controller)
    ball_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
