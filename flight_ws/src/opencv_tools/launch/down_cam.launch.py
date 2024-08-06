from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='opencv_tools',
            executable='down_image_publisher',
            name='down_image_publisher',
            output='screen',
            parameters=[{'param_name': 'param_value'}]
        ),
        Node(
            package='opencv_tools',
            executable='circle_ctr',
            name='right_qr_pub',
            output='screen',
            parameters=[{'param_name': 'param_value'}]
        ),
    ])

if __name__ == '__main__':
    generate_launch_description()