import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import time
import numpy as np
from geometry_msgs.msg import Point

class BallPIDController(Node):
    def __init__(self):
        super().__init__('ball_pid_controller')
        # Publishers for control efforts
        self.publisher_x = self.create_publisher(Float64, 'control_effort_x', 10)
        self.publisher_y = self.create_publisher(Float64, 'control_effort_y', 10)
        
        # Subscribers for actual ball positions

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

        self.timer_period = 1.0  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.test_prev_t = 0

    def map_range_x(x):
        return (x - 104) * (17.75 - -17.75) / (478 - 104) + (-17.75)   
    
    def map_range_y(y):
        return (y - 408.5) * (17.75 - -17.75) / (31.5 - 408.5) + -17.75
    
    def listener_callback(self, msg):
        self.process_value_x = self.map_range_x(msg.x) # Mapper fra piksler til coordinater i cm ut ifra størrelse på plata
        self.process_value_y = self.map_range_y(msg.y)

    def timer_callback(self):
        # PID Control for X
        current_t = time.time()
        if current_t - self.test_prev_t > 1:
            self.test_prev_t = current_t

        dt = current_t - self.test_prev_t
        control_output = np.sin((current_t - self.test_prev_t))

        error_x = self.setpoint_x - self.process_value_x
        self.integral_x += error_x * self.timer_period
        derivative_x = (error_x - self.last_error_x) / self.timer_period
        control_effort_x = (self.kp * error_x) + (self.ki * self.integral_x) + (self.kd * derivative_x)

        # PID Control for Y
        error_y = self.setpoint_y - self.process_value_y
        self.integral_y += error_y * self.timer_period
        derivative_y = (error_y - self.last_error_y) / self.timer_period
        control_effort_y = (self.kp * error_y) + (self.ki * self.integral_y) + (self.kd * derivative_y)

        # Publish control efforts
        msg_x = Float64()
        msg_x.data = control_effort_x
        self.publisher_x.publish(msg_x)

        msg_y = Float64()
        msg_y.data = control_effort_y
        self.publisher_y.publish(msg_y)

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
