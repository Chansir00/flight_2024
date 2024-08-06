import rclpy
from rclpy.node import Node
from std_msgs.msg import String,UInt8
import sys
import select

class DataDistributor(Node):
    def __init__(self):
        super().__init__('data_distributor')
        # 创建订阅者，订阅话题
        self.subscription = self.create_subscription(
            String,
            'qr_code_content',
            self.listener_callback,
            10
        )
        


        self.subscription = self.create_subscription(
            UInt8, 
            'circle_detected', 
            self.circle_listener_callback, 
            10)
        self.subscription  # 防止未使用变量警告
        # 创建发布者，发布命令
        self.command_publisher = self.create_publisher(
            String,
            'command',
            10
        )
        # 初始化键值对映射
        self.keys = ['A1', 'A2', 'A3', 'A4', 'A5', 'A6', 
                     'B1', 'B2', 'B3', 'B4', 'B5', 'B6', 
                     'C1', 'C2', 'C3', 'C4', 'C5', 'C6', 
                     'D1', 'D2', 'D3', 'D4', 'D5', 'D6']
        
        self.key_index = 0  # 当前键的索引
        self.data_map = {key: "--" for key in self.keys}  # 存储数据与键的映射，初始化为 "--"
        self.used_data = set()  # 记录已使用的数据
        self.query_data = False  # 是否查询数据的标志
        self.circle_detect = 199
        
        
 
    def circle_listener_callback(self, msg):
        self.circle_detect = msg.data
        if self.circle_detect == 200:
            self.query_data = True
            self.fill_unused_data()
            self.get_logger().info('进入查询数据模式')

    def listener_callback(self, msg):
        # 从话题消息中获取数据
        data = msg.data
        # 尝试将data转换为整数
        try:
            data = int(data)
        except ValueError:
            return
        
        # 检查数据是否已经被使用过
        if data in self.used_data:
            return
        
        # 分配数据到当前键
        if self.key_index < len(self.keys):
            current_key = self.keys[self.key_index]
            self.data_map[current_key] = data
            self.used_data.add(data)  # 标记数据已被使用
            # 更新键索引
            self.key_index += 1
            if self.key_index >= len(self.keys):
                self.key_index = 0  # 如果到达末尾，则重新开始
            # 立即刷新输出
            self.print_data_map()
        else:
            self.get_logger().warn('所有键都已分配数据，等待下一个数据')

    def fill_unused_data(self):
        unused_data = set(range(1, 25)) - self.used_data
        unused_data = list(unused_data)
        unused_data_index = 0
        for key in self.keys:
            if self.data_map[key] == '--' and unused_data_index < len(unused_data):
                self.data_map[key] = unused_data[unused_data_index]
                unused_data_index += 1
        self.print_data_map()
    
    def print_data_map(self):
        #self.get_logger().info('打印数据映射:')

        output = "\n-------------------------\n\n"
        output += f"\"D3\":{self.data_map['D3']},\"D2\":{self.data_map['D2']},\"D1\":{self.data_map['D1']},\n"
        output += f"\"D6\":{self.data_map['D6']},\"D5\":{self.data_map['D5']},\"D4\":{self.data_map['D4']},\n"
        output += f"\"C1\":{self.data_map['C1']},\"C2\":{self.data_map['C2']},\"C3\":{self.data_map['C3']},\n"
        output += f"\"C4\":{self.data_map['C4']},\"C5\":{self.data_map['C5']},\"C6\":{self.data_map['C6']},\n\n\n"
        output += f"\"B3\":{self.data_map['B3']},\"B2\":{self.data_map['B2']},\"B1\":{self.data_map['B1']},\n"
        output += f"\"B6\":{self.data_map['B6']},\"B5\":{self.data_map['B5']},\"B4\":{self.data_map['B4']},\n"
        output += f"\"A1\":{self.data_map['A1']},\"A2\":{self.data_map['A2']},\"A3\":{self.data_map['A3']},\n"
        output += f"\"A4\":{self.data_map['A4']},\"A5\":{self.data_map['A5']},\"A6\":{self.data_map['A6']},\n\n"
        output += "---------------------------"
        self.get_logger().info(output)

    def query_data_map(self):
        print("进入查询模式: 请输入数字 (1-24) 查询对应的键，或输入0退出查询:")
        if self.circle_detected == 200:

            while self.query_data:
                query_input = sys.stdin.readline().strip()
                if query_input == '0':
                    self.query_data = False
                else:
                    try:
                        self.query_number = int(query_input)
                        if 1 <= self.query_number <= 24:
                            found = False
                            for key, value in self.data_map.items():
                                if value == self.query_number:
                                    self.get_logger().info(f'数字 {self.query_number} 对应的键为: {key}')
                                    found = True
                                    break
                            if not found:
                                self.get_logger().info(f'未找到数字 {self.query_number} 对应的键')
                        else:
                            self.get_logger().info('请输入1到24之间的数字')
                    except ValueError:
                        self.get_logger().info('输入无效，请输入一个数字')

    def run(self):
        while rclpy.ok():
            if self.query_data:
                self.query_data_map()
            rclpy.spin_once(self)
            if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
                input_cmd = sys.stdin.readline().strip()
                if input_cmd == 'q':
                    self.query_data = True
                    self.get_logger().info('进入查询数据模式')

def main(args=None):
    rclpy.init(args=args)
    node = DataDistributor()
    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down DataDistributor node.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
