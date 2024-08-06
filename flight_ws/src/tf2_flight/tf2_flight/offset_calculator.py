import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
import tf2_ros

class OffsetCalculator(Node):
    def __init__(self):
        super().__init__('offset_calculator')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(0.1, self.calculate_offset)
        self.declare_parameter('robot_frame', 'base_link')
        self.robot_frame = self.get_parameter('robot_frame').get_parameter_value().string_value
        self.world_frame = 'world'

    def calculate_offset(self):
        try:
            # 获取机器人相对于世界坐标系的位置
            trans = self.tf_buffer.lookup_transform(self.world_frame, self.robot_frame, rclpy.time.Time())
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            z = trans.transform.translation.z
            self.get_logger().info(f'Offset: x={x}, y={y}, z={z}')
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            self.get_logger().warn('Could not transform')

def main(args=None):
    rclpy.init(args=args)
    node = OffsetCalculator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
