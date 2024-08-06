import rclpy
from rclpy.node import Node
from std_msgs.msg import String, UInt8
import sys
import select
from gpiozero import LED
from time import sleep


class DataDistributor(Node):
    def __init__(self):
        super().__init__('data_distributor')
        # Create subscribers for both topics
        self.subscription_left = self.create_subscription(
            String,
            'left_qr_code_content',
            self.listener_left_callback,
            10
        )
        self.subscription_right = self.create_subscription(
            String,
            'right_qr_code_content',
            self.listener_right_callback,
            10
        )
        self.subscription_left  # Prevent unused variable warning
        self.subscription_right  # Prevent unused variable warning

        self.led = LED(17)

        # Create publisher for commands
        self.command_publisher = self.create_publisher(
            String,
            'command',
            10
        )

        # Initialize key mappings
        self.left_keys = [
            'B1', 'B2', 'B3', 'B6', 'B5', 'B4',
            'D1', 'D2', 'D3', 'D6', 'D5', 'D4'
        ]
        self.right_keys = [
            'A3', 'A2', 'A1', 'A4', 'A5', 'A6',
            'C3', 'C2', 'C1', 'C4', 'C5', 'C6'
        ]

        self.left_key_index = 0  # Current key index for left data
        self.right_key_index = 0  # Current key index for right data
        self.left_data_map = {key: "--" for key in self.left_keys}  # Data map for left topic
        self.right_data_map = {key: "--" for key in self.right_keys}  # Data map for right topic
        self.used_data = set()  # Record used data
        self.query_data = False  # Flag for querying data
        self.circle_detect = 199

    def circle_listener_callback(self, msg):
        self.circle_detect = msg.data
        if self.circle_detect == 200:
            self.query_data = True
            self.fill_unused_data()
            self.get_logger().info('Entering query mode')

    def listener_left_callback(self, msg):
        self.process_data(msg.data, self.left_keys, self.left_data_map, self.left_key_index)
        self.left_key_index += 1
        if self.left_key_index >= len(self.left_keys):
            self.left_key_index = 0  # Reset index if at the end

    def listener_right_callback(self, msg):
        self.process_data(msg.data, self.right_keys, self.right_data_map, self.right_key_index)
        self.right_key_index += 1
        if self.right_key_index >= len(self.right_keys):
            self.right_key_index = 0  # Reset index if at the end

    def process_data(self, data, keys, data_map, key_index):
        try:
            data = int(data)
        except ValueError:
            return

        if data in self.used_data:
            return

        if key_index < len(keys):
            current_key = keys[key_index]
            data_map[current_key] = data
            self.used_data.add(data)
            self.print_data_maps()

    def fill_unused_data(self):
        unused_data = set(range(1, 25)) - self.used_data
        unused_data = list(unused_data)
        unused_data_index = 0
        for key in self.left_keys:
            if self.left_data_map[key] == '--' and unused_data_index < len(unused_data):
                self.left_data_map[key] = unused_data[unused_data_index]
                unused_data_index += 1
        for key in self.right_keys:
            if self.right_data_map[key] == '--' and unused_data_index < len(unused_data):
                self.right_data_map[key] = unused_data[unused_data_index]
                unused_data_index += 1
        self.print_data_maps()

    def print_data_maps(self):
        output = "Current warehouse status:"
        output += "\n-------------------------\n\n"
        output += f"Left Data Map:\n"
        output += f"\"D3\":{self.left_data_map['D3']}, \"D2\":{self.left_data_map['D2']}, \"D1\":{self.left_data_map['D1']},\n"
        output += f"\"D6\":{self.left_data_map['D6']}, \"D5\":{self.left_data_map['D5']}, \"D4\":{self.left_data_map['D4']},\n"
        output += f"\"B3\":{self.left_data_map['B3']}, \"B2\":{self.left_data_map['B2']}, \"B1\":{self.left_data_map['B1']},\n"
        output += f"\"B6\":{self.left_data_map['B6']}, \"B5\":{self.left_data_map['B5']}, \"B4\":{self.left_data_map['B4']},\n\n\n"
        output += f"Right Data Map:\n"
        output += f"\"C1\":{self.right_data_map['C1']}, \"C2\":{self.right_data_map['C2']}, \"C3\":{self.right_data_map['C3']},\n"
        output += f"\"C4\":{self.right_data_map['C4']}, \"C5\":{self.right_data_map['C5']}, \"C6\":{self.right_data_map['C6']},\n"
        output += f"\"A1\":{self.right_data_map['A1']}, \"A2\":{self.right_data_map['A2']}, \"A3\":{self.right_data_map['A3']},\n"
        output += f"\"A4\":{self.right_data_map['A4']}, \"A5\":{self.right_data_map['A5']}, \"A6\":{self.right_data_map['A6']},\n\n"
        output += "---------------------------"
        self.get_logger().info(output)
        self.led.off()
        sleep(1)
        self.led.on()

    def query_data_map(self):
        print("Entering query mode: Enter cargo number (1-24) to query the corresponding coordinates, or enter 0 to exit query:")
        if self.circle_detect == 200:
            while self.query_data:
                query_input = sys.stdin.readline().strip()
                if query_input == '0':
                    self.get_logger().info('Exiting query mode')
                    self.query_data = False
                else:
                    try:
                        query_number = int(query_input)
                        if 1 <= query_number <= 24:
                            found = False
                            for key, value in self.left_data_map.items():
                                if value == query_number:
                                    self.get_logger().info(f'Cargo {query_number} corresponds to coordinate: {key} (Left)')
                                    found = True
                                    break
                            for key, value in self.right_data_map.items():
                                if value == query_number:
                                    self.get_logger().info(f'Cargo {query_number} corresponds to coordinate: {key} (Right)')
                                    found = True
                                    break
                            if not found:
                                self.get_logger().info(f'No corresponding coordinates found for cargo {query_number}')
                        else:
                            self.get_logger().info('Please enter a number between 1 and 24')
                    except ValueError:
                        self.get_logger().info('Invalid input, please enter a number')

    def run(self):
        while rclpy.ok():
            if self.query_data:
                self.query_data_map()
            rclpy.spin_once(self)
            if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
                input_cmd = sys.stdin.readline().strip()
                if input_cmd == 'q':
                    self.query_data = True
                    self.get_logger().info('Entering query data mode')

def main(args=None):
    rclpy.init(args=args)
    node = DataDistributor()
    node.led.off()
    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down DataDistributor node.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
