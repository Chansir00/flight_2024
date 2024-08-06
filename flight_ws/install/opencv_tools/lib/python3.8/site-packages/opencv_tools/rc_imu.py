import rclpy                                     # ROS2 Python Client Library
from rclpy.node import Node                      # ROS2 Node
from sensor_msgs.msg import Imu

# Usart Library
import serial
import struct
import binascii

'''
    Author: Liu Yuxiang 
    Time: 2022.12.13
    description: IMU底层串口收发代码
'''


# imu接收数据类型
class IMU(Node):
    send_data = []
    def __init__(self):
        super().__init__('imu_publisher')
        self.publisher_ = self.create_publisher(Imu, 'imu_data', 1)

        # 串口初始化
        self.IMU_Usart = serial.Serial(
            port = '/dev/ttyTHS0',      # 串口
            baudrate=115200,            # 波特率
            timeout = 0.001             # 由于后续使用read_all按照一个timeout周期时间读取数据
                                        # imu在波特率115200返回数据时间大概是1ms,9600下大概是10ms
                                        # 所以读取时间设置0.001s
        )
        # 接收数据初始化
        self.ACC_X:float = 0.0                   # X轴加速度
        self.ACC_Y:float = 0.0                   # Y轴加速度
        self.ACC_Z:float  =  0.0                 # Z轴加速度
        self.GYRO_X :float = 0.0                 # X轴陀螺仪
        self.GYRO_Y :float = 0.0                 # Y轴陀螺仪
        self.GYRO_Z :float = 0.0                 # Z轴陀螺仪
        self.Q0 :float = 0.0                     # 四元数Q0.0
        self.Q1 :float = 0.0                     # 四元数Q1
        self.Q2 :float = 0.0                     # 四元数Q2
        self.Q3 :float = 0.0                     # 四元数Q3
        # 判断串口是否打开成功
        if self.IMU_Usart.isOpen():
            print("open success")
        else:
            print("open failed")

        # 回调函数返回周期
        time_period = 0.01
        self.timer = self.create_timer(time_period, self.timer_callback)

        # 发送读取指令
        #self.Send_ReadCommand()

    def Send_ReadCommand(self):
        '''
        Author: Liu Yuxiang 
        Time: 2022.12.13
        description: 发送读取IMU内部数据指令
        · 第一个寄存器0x08 最后一个读取寄存器0x2A 共35个
        · 读寄存器例子,读取模块内部温度,主站发送帧为:A4 03 1B 02 C4
            |   A4    |    03    |    1B   |     02    |    C4
            |  帧头ID  | 读功能码 |起始寄存器| 寄存器数量 |校验和低 8 位
        '''
        # 使用优雅的方式发送串口数据
        send_data = [0xA4,0x03,0x08,0x23,0xD2]                      #需要发送的串口包
        #send_data = [0xA4,0x03,0x08,0x1B,0xCA]                      #需要发送的串口包
        send_data=struct.pack("%dB"%(len(send_data)),*send_data)    #解析成16进制
        print(send_data)
        self.IMU_Usart.write(send_data)                             #发送


    # def Read_data(self):

    #     # 初始化数据
    #     counter = 0 
    #     Recv_flag = 0
    #     Read_buffer = []
    #     # 接收数据至缓存区
    #     Read_buffer = self.IMU_Usart.read(40)  # 读取40个字节
        
    #     # 状态机判断收包数据是否准确
    #     while True:
    #         # 第1帧是否是帧头ID 0xA4
    #         if counter == 0:
    #             if Read_buffer[0] != 0xA4:
    #                 break    

    #         # 第2帧是否是读功能码 0x03  
    #         elif counter == 1:
    #             if Read_buffer[1] != 0x03:
    #                 counter = 0
    #                 break

    #         # 第3帧判断起始帧        
    #         elif counter == 2:
    #             if Read_buffer[2] < 0x2c:
    #                 start_reg = Read_buffer[2]
    #             else:
    #                 counter = 0 

    #         # 第4帧判断帧有多少数量 
    #         elif counter == 3:
    #             if (start_reg + Read_buffer[3]) < 0x2C:  # 最大寄存器为2C，大于0x2C说明数据错了
    #                 length = Read_buffer[3]
    #             else:
    #                 counter = 0
    #                 break
            
    #         # 判断数据包是否完整
    #         else:
    #             if len(Read_buffer) >= length + 5 and counter == length + 5:
    #                 Recv_flag = 1

    #         # 收包完毕
    #         if Recv_flag:
    #             Recv_flag = 0
    #             sum = 0
    #             data_inspect = str(binascii.b2a_hex(Read_buffer))
                
    #             try:
    #                 # 检验所有帧之和低八位是否等于末尾帧
    #                 for i in range(2, 80, 2):
    #                     sum += int(data_inspect[i:i+2], 16)
                    
    #                 if str(hex(sum))[-2:] == data_inspect[80:82]:  # 数据检验通过
    #                     unpack_data = struct.unpack('<hhh hhh hhhhh', Read_buffer[4:-1])
                        
    #                     g = 9.8
    #                     # 解析所需的数据
    #                     self.ACC_X = unpack_data[0] / 2048 * g  # 单位 m/s^2
    #                     self.ACC_Y = unpack_data[1] / 2048 * g
    #                     self.ACC_Z = unpack_data[2] / 2048 * g
    #                     self.GYRO_X = unpack_data[3] / 16.4     # 单位 度/s
    #                     self.GYRO_Y = unpack_data[4] / 16.4
    #                     self.GYRO_Z = unpack_data[5] / 16.4
    #                     self.Q0 = unpack_data[14] / 10000
    #                     self.Q1 = unpack_data[15] / 10000
    #                     self.Q2 = unpack_data[16] / 10000
    #                     self.Q3 = unpack_data[17] / 10000
    #                     print("recive:")
    #             except Exception as e:
    #                 print("接收数据时发生错误:", e)
    #             counter = 0
    #             break
    #         else:
    #             counter += 1

    def Read_data(self):
        # 初始化数据
        Read_buffer = self.IMU_Usart.read(22)  # 读取22个字节
        
        # 检查帧头ID和功能码
        if Read_buffer[0] != 0xA4 or Read_buffer[1] != 0x03:
            print(Read_buffer[0])
            print("帧头ID或功能码不正确")
            return

        # 解析数据
        try:
            # 只解析所需的10个字段
            unpack_data = struct.unpack('<hhh hhh hhhh', Read_buffer[2:22])
            
            g = 9.8
            # 解析所需的数据
            self.ACC_X = unpack_data[0] / 2048 * g  # 单位 m/s^2
            self.ACC_Y = unpack_data[1] / 2048 * g
            self.ACC_Z = unpack_data[2] / 2048 * g
            self.GYRO_X = unpack_data[3] / 16.4     # 单位 度/s
            self.GYRO_Y = unpack_data[4] / 16.4
            self.GYRO_Z = unpack_data[5] / 16.4
            self.Q0 = unpack_data[6] / 10000
            self.Q1 = unpack_data[7] / 10000
            self.Q2 = unpack_data[8] / 10000
            self.Q3 = unpack_data[9] / 10000
            
            print("接收到的数据:")
            print(f"ACC_X: {self.ACC_X}, ACC_Y: {self.ACC_Y}, ACC_Z: {self.ACC_Z}")
            print(f"GYRO_X: {self.GYRO_X}, GYRO_Y: {self.GYRO_Y}, GYRO_Z: {self.GYRO_Z}")
            print(f"Q0: {self.Q0}, Q1: {self.Q1}, Q2: {self.Q2}, Q3: {self.Q3}")
        except Exception as e:
            print("接收数据时发生错误:", e)


        

    def timer_callback(self):
        try:
            count = self.IMU_Usart.inWaiting()
            if count > 0:
                self.Read_data()
            
            # 发布 sensor_msgs/Imu 数据类型
            imu_data = Imu()
            imu_data.header.frame_id = "imu_link"  # 或者使用其他合适的 frame_id
            imu_data.header.stamp = self.get_clock().now().to_msg()
            imu_data.linear_acceleration.x = self.ACC_X
            imu_data.linear_acceleration.y = self.ACC_Y
            imu_data.linear_acceleration.z = self.ACC_Z
            imu_data.angular_velocity.x = self.GYRO_X * 3.1415926 / 180.0  # 转换为 rad/s
            imu_data.angular_velocity.y = self.GYRO_Y * 3.1415926 / 180.0
            imu_data.angular_velocity.z = self.GYRO_Z * 3.1415926 / 180.0
            imu_data.orientation.x = self.Q0
            imu_data.orientation.y = self.Q1
            imu_data.orientation.z = self.Q2
            imu_data.orientation.w = self.Q3
            self.publisher_.publish(imu_data)

        except KeyboardInterrupt:
            if self.IMU_Usart:
                print("关闭串口")
                self.IMU_Usart.close()



def main(args=None):

    # 变量初始化---------------------------------------------    
    rclpy.init(args=args)
    IMU_node = IMU()
    rclpy.spin(IMU_node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()


'''
░░░░░░░░░░░░░░░░░░░░░░░░▄░░
░░░░░░░░░▐█░░░░░░░░░░░▄▀▒▌░
░░░░░░░░▐▀▒█░░░░░░░░▄▀▒▒▒▐░
░░░░░▄▄▀▒░▒▒▒▒▒▒▒▒▒█▒▒▄█▒▐░
░░░▄▀▒▒▒░░░▒▒▒░░░▒▒▒▀██▀▒▌░
░░▐▒▒▒▄▄▒▒▒▒░░░▒▒▒▒▒▒▒▀▄▒▒░
░░▌░░▌█▀▒▒▒▒▒▄▀█▄▒▒▒▒▒▒▒█▒▐
░▐░░░▒▒▒▒▒▒▒▒▌██▀▒▒░░░▒▒▒▀▄
░▌░▒▄██▄▒▒▒▒▒▒▒▒▒░░░░░░▒▒▒▒
▀▒▀▐████▌▄░▀▒▒░░░░░░░░░░▒▒▒
狗狗保佑代码无bug
'''
