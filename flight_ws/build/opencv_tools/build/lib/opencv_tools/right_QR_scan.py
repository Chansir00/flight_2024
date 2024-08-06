import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import Point
import cv2
import numpy as np
from cv_bridge import CvBridge
from pyzbar.pyzbar import decode
import RPi.GPIO as GPIO
import time



led_pin = 12 #32

GPIO.setmode(GPIO.BOARD)  # BOARD pin-numbering scheme
GPIO.setup(led_pin, GPIO.OUT)
GPIO.output(led_pin, GPIO.LOW)


class RightQRCodeReader(Node):
    def __init__(self):
        super().__init__('right_qr_code_reader')
        self.subscription = self.create_subscription(
            Image,
            'down_video_frames',
            self.image_callback,
            10
        )
        self.publisher_content = self.create_publisher(String, 'right_qr_code_content', 10)
        self.publisher_position = self.create_publisher(Point, 'right_qr_code_position', 10)
        self.bridge = CvBridge()
        self.get_logger().info('QR Code Reader node has been started.')
        # self.timer = self.create_timer(0.1, self.timer_callback)  # Timer to call periodically

    def image_callback(self, msg):
    # try:
        # Convert ROS Image message to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        # Preprocess the image
        preprocessed_image = self.preprocess_image(cv_image)

        # Decode QR codes
        decoded_objects = decode(preprocessed_image)

        if decoded_objects:
            for obj in decoded_objects:
                data = obj.data.decode('utf-8')
                if data:
                    #self.get_logger().info(f'Detected QR code with content: {data}')
                    # 发布QR码内容
                    qr_code_msg = String()
                    qr_code_msg.data = data
                    self.publisher_content.publish(qr_code_msg)

                    # 计算并发布QR码的中心
                    points = obj.polygon
                    if len(points) == 4:
                        center_x = float((points[0][0] + points[2][0]) / 2)
                        center_y = float((points[0][1] + points[2][1]) / 2)
                        qr_code_position_msg = Point()
                        qr_code_position_msg.x = center_x
                        qr_code_position_msg.y = center_y
                        qr_code_position_msg.z = 0.0
                        self.publisher_position.publish(qr_code_position_msg)
                        self.control_led(center_x, center_y)
        else:
            # 如果没有检测到QR码，关闭LED灯
            center_x = 0
            center_y = 0
            GPIO.output(led_pin, GPIO.LOW)
        cv2.imshow('result',preprocessed_image)
        cv2.waitKey(1)


            # Display the image with QR code detection

        # except Exception as e:
        #     self.get_logger().error(f'Error processing image: {e}')

    def preprocess_image(self, cv_image):
        # Convert to grayscale
        gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        # Apply thresholding to get a binary image
        _, binary_image = cv2.threshold(gray_image, 128, 255, cv2.THRESH_BINARY)
        return binary_image
    

    def control_led(self, center_x, center_y):
        # 定义中心区域的容忍度
        center_tolerance = 100
        frame_center_x = 320  # 假设图像的宽度为640
        frame_center_y = 240  # 假设图像的高度为480

        # 检查QR码是否在中心区域
        if (frame_center_x - center_tolerance < center_x < frame_center_x + center_tolerance or
                frame_center_y - center_tolerance < center_y < frame_center_y + center_tolerance):
            GPIO.output(led_pin, GPIO.HIGH)
        else:
            GPIO.output(led_pin, GPIO.LOW)

def main(args=None):
    rclpy.init(args=args)
    node = RightQRCodeReader()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down QR Code Reader node.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
