from math import sin, cos, pi
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster, TransformStamped
from std_msgs.msg import Float64MultiArray
import numpy as np

class StatePublisher(Node):

    def __init__(self):
        super().__init__('state_publisher')

        qos_profile = QoSProfile(depth=10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', qos_profile)
        self.broadcaster = TransformBroadcaster(self, qos=qos_profile)
        self.nodeName = self.get_name()
        self.get_logger().info("{0} started".format(self.nodeName))
        #Creates a subscriber
        self.shoulder_pan_joint_subscriber = self.create_subscription(
            Float64MultiArray,
            'angle_outputs',
            self.topic_callback,
            10)
        #Initializing the angle values
        self.angle1 = 0.0
        self.angle2 = 0.0
        

        self.odom_trans = TransformStamped()
        self.odom_trans.header.frame_id = 'stamme_link'
        self.odom_trans.child_frame_id = 'plate_link'
        self.joint_state = JointState()
        #Timer which runs the transform_callback 30 times per second
        self.create_timer(1.0 / 30, self.transform_callback) 
        #Callback which sets the Angle value to the actual roll and pitch angles from the topic
    def topic_callback(self, message):
        self.angle1 = np.deg2rad(message.data[0])
        self.angle2 = np.deg2rad(message.data[1])
        #Updates the transformations and publishes
    def transform_callback(self):
        now = self.get_clock().now()
        self.joint_state.header.stamp = now.to_msg()
        self.joint_state.name = ['swivel']
        self.joint_state.position = [self.angle1]
        
        self.odom_trans.header.stamp = now.to_msg()
        self.odom_trans.transform.translation.x = 0.0
        self.odom_trans.transform.translation.y = 0.0
        self.odom_trans.transform.translation.z = 0.0
        self.odom_trans.transform.rotation = euler_to_quaternion(self.angle1, self.angle2, 0)  # roll, pitch, yaw
        #Publishes the joint_states after the transformation
        self.joint_pub.publish(self.joint_state)
        self.broadcaster.sendTransform(self.odom_trans)
        
def euler_to_quaternion(roll, pitch, yaw):
    qx = sin(roll / 2) * cos(pitch / 2) * cos(yaw / 2) - cos(roll / 2) * sin(pitch / 2) * sin(yaw / 2)
    qy = cos(roll / 2) * sin(pitch / 2) * cos(yaw / 2) + sin(roll / 2) * cos(pitch / 2) * sin(yaw / 2)
    qz = cos(roll / 2) * cos(pitch / 2) * sin(yaw / 2) - sin(roll / 2) * sin(pitch / 2) * cos(yaw / 2)
    qw = cos(roll / 2) * cos(pitch / 2) * cos(yaw / 2) + sin(roll / 2) * sin(pitch / 2) * sin(yaw / 2)
    return Quaternion(x=qx, y=qy, z=qz, w=qw)

def main():
    rclpy.init()
    node = StatePublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
