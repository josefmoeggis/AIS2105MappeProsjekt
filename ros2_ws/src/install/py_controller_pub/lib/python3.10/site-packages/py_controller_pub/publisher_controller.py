import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

class MotorCommandPublisher(Node):
    def __init__(self):
        super().__init__('motor_command_publisher')
        self.publisher = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)
        self.subscription = self.create_subscription(
            Float64MultiArray,
            'motor_outputs',
            self.listener_callback,
            10
        )
        self.motor_values = [0.0, 0.0, 0.0]  # Default values
        self.timer = self.create_timer(1.0, self.timer_callback)

    def listener_callback(self, msg):
        self.motor_values = msg.data
        self.get_logger().info(f'Received motor values: {self.motor_values}')

    def timer_callback(self):
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = ['ard_motorA', 'ard_motorB', 'ard_motorC']
        point = JointTrajectoryPoint()
        point.positions = self.motor_values
        point.time_from_start = Duration(sec=1, nanosec=0)
        msg.points.append(point)
        self.publisher.publish(msg)
        self.get_logger().info('Publishing motor commands')

def main(args=None):
    rclpy.init(args=args)
    motor_command_publisher = MotorCommandPublisher()
    rclpy.spin(motor_command_publisher)
    motor_command_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
