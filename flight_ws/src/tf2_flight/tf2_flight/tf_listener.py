#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException
from rclpy.time import Time
from geometry_msgs.msg import Point

class TfListenerNode(Node):
    def __init__(self):
        super().__init__('tf_listener_node')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(1.0, self.timer_callback)
        
        # 创建一个发布器来发布 xy 坐标
        self.publisher = self.create_publisher(Point, 'xy_coordinates', 10)

    def timer_callback(self):
        try:
            now = Time()
            trans = self.tf_buffer.lookup_transform('map', 'base_link', now)
            
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            
            self.get_logger().info(f'Current position: x={x}, y={y}')
            
            # 创建并发布 Point 消息
            point_msg = Point()
            point_msg.x = x
            point_msg.y = y
            point_msg.z = 0.0  # z 坐标设置为 0.0，因为你只需要 xy 坐标
            
            self.publisher.publish(point_msg)
            
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f'Could not transform: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = TfListenerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
