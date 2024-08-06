import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8
from sensor_msgs.msg import LaserScan
from queue import Queue
import serial
import numpy as np
import threading  # Import threading module for locks

class SerialPublisher(Node):
    def __init__(self):
        super().__init__('serial_publisher')
        self.stick_detected = 0xc7  # Default value for stick_detected
        self.circle_detected = 0xc7  # Default value for circle_detected
        self.distance = 0x00  # Default value for distance
        self.circle_position = 0xc7  # Default value for circle_position
        self.stick_position = 0xc7  # Default value for stick_position

        self.lock = threading.Lock()  # Create a lock
        self.queue = Queue()  # Create a queue for handling send requests
        
        self.stick_subscription = self.create_subscription(
            UInt8,
            'stick_detected',
            self.stick_listener_callback,
            10)
        
        self.circle_subscription = self.create_subscription(
            UInt8,
            'circle_detected',
            self.circle_listener_callback,
            10)
        
        self.scan_subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_listener_callback,
            10
        )

        self.circle_ctr_subscription = self.create_subscription(
            UInt8,
            'circle_position',
            self.circle_ctr_listener_callback,
            10
        )

        self.stick_ctr_subscription = self.create_subscription(
            UInt8,
            'stick_position',
            self.stick_ctr_listener_callback,
            10
        )

        # Start a thread to handle sending data
        self.send_thread = threading.Thread(target=self.send_data_loop)
        self.send_thread.daemon = True
        self.send_thread.start()
        
        # Initialize the serial connection
        self.ser = serial.Serial(
            port='/dev/ttyUSB0',  # Update with the correct serial port
            baudrate=9600,
            timeout=1
        )
        
    def stick_listener_callback(self, msg):
        self.get_logger().info(f'Received stick_detected: {msg.data}')
        with self.lock:
            self.stick_detected = msg.data
        self.queue.put('send')
        
    def circle_listener_callback(self, msg):
        self.get_logger().info(f'Received circle_detected: {msg.data}')
        with self.lock:
            self.circle_detected = msg.data
        self.queue.put('send')

    def scan_listener_callback(self, msg):
        front_index = 0
        distances = [msg.ranges[44], msg.ranges[45], msg.ranges[46]]
        valid_distances = [d for d in distances if not np.isnan(d)]

        if valid_distances:
            min_distance = min(valid_distances)
        else:
            min_distance = float('inf')  # 或者其他合适的默认值

        self.get_logger().info(f'Received scan: min_distance in 44-46 degrees: {min_distance:.3f} meters')
        self.distance = int(1000*min_distance) 
        # with self.lock:  # Acquire the lock before modifying shared data
        #       # Convert to millimeters
        #self.queue.put('send')  # Put send request in the queue

    def circle_ctr_listener_callback(self, msg):
        self.get_logger().info(f'Received circle_position: {msg.data}')
        with self.lock:
            self.circle_position = msg.data
        self.queue.put('send')

    def stick_ctr_listener_callback(self, msg):
        self.get_logger().info(f'Received stick_position: {msg.data}')
        with self.lock:
            self.stick_position = msg.data
        self.queue.put('send')

    def send_data_loop(self):
        while rclpy.ok():
            self.queue.get()  # Wait for a send request
            self.send_data()

    def send_data(self):
        try:
            # Create the data array with header and footer
            header = 0xAA  # Example header byte
            footer = 0x55  # Example footer byte
            with self.lock:
                data = [header, self.stick_detected, self.circle_detected, self.distance, self.stick_position,self.circle_position ,footer]
            
            # Send the data array over the serial connection
            self.ser.write(bytearray(data))
            self.get_logger().info(f'Sent data: {data}')
        except Exception as e:
            self.get_logger().error(f'Serial write failed: {e}')

def main(args=None):
    rclpy.init(args=args)
    serial_publisher = SerialPublisher()
    rclpy.spin(serial_publisher)
    serial_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
