import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8, UInt8MultiArray,UInt16,String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point
import serial
import numpy as np  
import struct

class SerialPublisher(Node):
    def __init__(self):
        super().__init__('serial_publisher')
        self.QR_num=0
        self.circle_detected = 0xc7  # Default value for circle_detected
        self.distances = [0,0,0,0]
        self.circle_ctr_x = 0x00
        self.circle_ctr_y = 0x00
        self.x=[0,0]
        self.y=[0,0]
        self.last_data = None
        self.received_data = []  # 用于存储已接收的唯一数据
        
        # self.stick_subscription = self.create_subscription(
        #     UInt8,
        #     'stick_detected',
        #     self.stick_listener_callback,
        #     10)
        
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

        self.QR_ctr_subscription= self.create_subscription(
            Point, 'qr_code_position',self.QR_ctr_listener_callback, 10
        )

        self.subscription = self.create_subscription(
            String,
            'qr_code_content',
            self.qr_listener_callback,
            10
        )
        self.subscription  # 防止未使用变量警告

        # Initialize the serial connection
        self.ser = serial.Serial(
            port='/dev/ttyTHS0',  # Update with the correct serial port
            baudrate=115200,
            timeout=1
        )
        self.subscription

    def stick_listener_callback(self, msg):
        #self.get_logger().info(f'Received stick_detected: {msg.data}')
        self.stick_detected = msg.data
        #self.send_data()
        
    def circle_listener_callback(self, msg):
        #self.get_logger().info(f'Received circle_detected: {msg.data}')
        self.circle_detected = msg.data
        #self.send_data()


    # def scan_listener_callback(self, msg):
    #     front_index = 270
    #     self.get_logger().info('Received scan: {:.2f}'.format(msg.ranges[front_index]))
    #     self.distance = round(msg.ranges[front_index], 3)
        
    #     self.distance=int(self.distance*100)
    #     self.send_data()


    def scan_listener_callback(self, msg):
        # 初始化最小距离为一个很大的值
        min_distance_front = float('inf')
        min_distance_right = float('inf')
        min_distance_left = float('inf')
        min_distance_back = float('inf')

        # 前方向
        for i in range(105, 145):
            distance = msg.ranges[i]
            if not np.isnan(distance) and distance < min_distance_front:
                min_distance_front = distance
        self.distances[0] = min_distance_front

        # 右方向
        for i in range(0, 35):
            distance = msg.ranges[i]
            if not np.isnan(distance) and distance < min_distance_right:
                min_distance_right = distance
        self.distances[1] = min_distance_right

        # 左方向
        for i in range(225, 265):
            distance = msg.ranges[i]
            if not np.isnan(distance) and distance < min_distance_left:
                min_distance_left = distance
        self.distances[2] = min_distance_left

        # 后方向
        for i in range(355, 395):
            distance = msg.ranges[i]
            if not np.isnan(distance) and distance < min_distance_back:
                min_distance_back = distance
        self.distances[3] = min_distance_back



            # 遍历所有距离值，找出最小的非 NaN 值
            # for distance in msg.ranges:
            #     if not np.isnan(distance) and distance < min_distance:
            #         min_distance = distance

            # 如果最小距离仍然是无限大，说明所有值都是 NaN
        for i in range(len(self.distances)):
            if self.distances[i] == float('inf'):
                #self.get_logger().warning('Received all NaN values in scan')
                self.distances[i] = 0.0  # 你可以选择一个合适的默认值，或跳过更新
            else:
                #self.get_logger().info('Received minimum scan distance: {:.2f}'.format(min_distance))
                self.distances[i] = round(self.distances[i], 3)
                
                if  self.distances[i] > 2.5:
                    self.distances[i] = 0
                else:
                    self.distances[i] = int(self.distances[i]*100)
        #self.distance = min_distance # 将最小距离转换为整数（假设是以厘米为单位）
        self.send_data()     

    def circle_ctr_listener_callback(self, msg):
        #self.get_logger().info(f'Received circle_position: {msg.data}')
        # with self.lock:  # Acquire the lock before modifying shared data
        self.circle_ctr_x = int(round(msg.x,2)*100)
        self.circle_ctr_y = int(round(msg.y,2)*100)
        # self.x = int(x *1000)
        # self.y = int(y *1000)
        # 将 x 和 y 转换为 uint16，然后再分解为两个 uint8 字节
        x_bytes = struct.pack('h', self.circle_ctr_x)
        y_bytes = struct.pack('h', self.circle_ctr_y)
        
        self.x = [x_bytes[0], x_bytes[1]]
        self.y= [y_bytes[0], y_bytes[1]]
        #self.send_data()

    def QR_ctr_listener_callback(self, msg):
        #self.get_logger().info(f'Received circle_position: {msg.data}')
        # with self.lock:  # Acquire the lock before modifying shared data
        self.qr_ctr_x = int(msg.x)
        self.qr_ctr_y = int(msg.y)
        self.qr_ctr_x_high = (self.qr_ctr_x >> 8) & 0xFF
        self.qr_ctr_x_low = self.qr_ctr_y & 0xFF
        # y_high_byte = (y >> 8) & 0xFF
        # y_low_byte = y & 0xFF



    def qr_listener_callback(self, msg):
        # 将String消息中的data转换为整数
        try:
            data = int(msg.data)
        except ValueError:
            # 如果转换失败，记录警告并返回
            self.get_logger().warn(f'Invalid data received: {msg.data}')
            return
        
        # 检查数据是否已经被接收过
        if data not in self.received_data:
            self.received_data.append(data)  # 添加到接收数据列表中
            self.QR_num += 1
            #self.get_logger().info(f'New QR_num: {self.QR_num}, Received Data: {self.received_data}')
        #else:
            #self.get_logger().info(f'Data {data} has already been received.')

        #plane_tf
    # def listener_callback(self, msg):
    #     x = int(round(msg.x,2)*100)
    #     y = int(round(msg.y,2)*100)
    #     # self.x = int(x *1000)
    #     # self.y = int(y *1000)
    #     # 将 x 和 y 转换为 uint16，然后再分解为两个 uint8 字节
    #     x_bytes = struct.pack('h', x)
    #     y_bytes = struct.pack('h', y)
        
    #     self.x = [x_bytes[0], x_bytes[1]]
    #     self.y= [y_bytes[0], y_bytes[1]]
        #self.get_logger().info(f'Received coordinates: x={self.x}, y={self.y}')

        

    def send_data(self):
            try:
                header = 0xAA # Example header bytes
                footer = 0x55  # Example footer bytes
                
                # Increment the frame counter
                # self.Frame_Cnt += 1
                # if self.Frame_Cnt > 65535:
                #     self.Frame_Cnt = 0
                
                # Calculate low and high bytes of the frame counter
                # cnt_low = self.Frame_Cnt & 0xFF
                # cnt_high = (self.Frame_Cnt >> 8) & 0xFF
                
                # Create the data array with header, counter, data, and footer
                #data = header ,self.QR_num,self.circle_detected,self.distance,self.qr_ctr_x_low,self.qr_ctr_x_high, self.x[0],self.x[1],self.y[0],self.y[1], footer
                data = header ,self.QR_num,self.circle_detected,self.distances[0],self.distances[2],self.distances[1],self.distances[3],self.x[0],self.x[1],self.y[0],self.y[1], footer 
                # Send the data array over the serial connection
                self.ser.write(bytearray(data))
                self.get_logger().info(f'Sent data: {data}')  # Use print for logging in this example
            except Exception as e:
                self.get_logger().info(f'Serial write failed: {e}')  # Use print for error logging in this example
            
     

def main(args=None):
    rclpy.init(args=args)
    serial_publisher = SerialPublisher()
    rclpy.spin(serial_publisher)
    serial_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
