import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tf2_flight',
            executable='pose_pub',
            name='iose_pub',
            output='screen',
            parameters=[{'param_name': 'param_value'}]
        ),
        Node(
            package='tf2_flight',
            executable='tf_listener',
            name='tf_listener',
            output='screen',
            parameters=[{'param_name': 'param_value'}]
        )
    ])
