from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='opencv_tools',
            executable='img_publisher',
            name='img_pubscriber',
            output='screen',
            parameters=[{'param_name': 'param_value'}]  # 如果有参数需要传递，可以在这里设置
        ),
        Node(
            package='opencv_tools',
            executable='left_qr_pub',
            name='qrscan',
            output='screen',
            parameters=[{'param_name': 'param_value'}]
        ),
        Node(
            package='opencv_tools',
            executable='to_img_mcu',
            name='to_img_mcu',
            output='screen',
            parameters=[{'param_name': 'param_value'}]
        ),
        # Node(
        #     package='opencv_tools',
        #     executable='right_qr_pub',
        #     name='right_qr_pub',
        #     output='screen',
        #     parameters=[{'param_name': 'param_value'}]
        # ),
        # Node(
        #     package='opencv_tools',
        #     executable='down_image_publisher',
        #     name='down_image_publisher',
        #     output='screen',
        #     parameters=[{'param_name': 'param_value'}]
        # ),
        # Node(
        #     package='opencv_tools',
        #     executable='sticker_ctr',
        #     name='sticker_ctr',
        #     output='screen',
        #     parameters=[{'param_name': 'param_value'}]
        # ),
    ])

if __name__ == '__main__':
    generate_launch_description()
