from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([        
        Node(
            package='our_arduino_driver',
            executable='our_arduino_driver',
            name='our_arduino_driver',
            output='screen'
        ),
        Node(
            package='magnus_kinematic',
            executable='new_controller',
            name='new_controller',
            output='screen'
        ),
        Node(
            package='canny_edge',
            executable='canny_edge_node',
            name='canny_edge_node',
            output='screen'
        ),
        Node(
            package='undistort_image',
            executable='undistort_image_node',
            name='undistort_image_node',
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
