from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='py_controller_pub',
            executable='arduino_controller',
            name='arduino_controller',
            output='screen'
        ),
        Node(
            package='py_kinematics',
            executable='kinematic_control',
            name='kinematic_controller',
            output='screen'
        ),
        Node(
            package='py_pid',
            executable='ball_control_pid',
            name='ball_control_pid',
            output='screen'
        ),
        Node(
            package='cv2_contour',
            executable='cv2_contour_node',
            name='cv2_contour_node',
            output='screen'
        ),
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam_node',
            output='screen',
            parameters=[{'video_device': '/dev/video2'}]
        ),
        Node(
            package='gaussian_blur',
            executable='gaussian_blur_node',
            name='gaussian_blur',
            output='screen'
        )
    ])
