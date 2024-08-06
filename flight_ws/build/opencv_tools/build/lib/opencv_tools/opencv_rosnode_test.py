import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from std_msgs.msg import UInt8
from cv_bridge import CvBridge

import numpy as np

stick_aspect_ratio = 3
msg = UInt8()
class OpencvSubscriber(Node):
    """
    Create an ImageSubscriber class, which is a subclass of the Node class.
    """
    def __init__(self):
        """
        Class constructor to set up the node
        """
        # Initiate the Node class's constructor and give it a name
        super().__init__('opencv_subsriber')
        
        # Create the subscriber. This subscriber will receive an Image
        # from the video_frames topic. The queue size is 10 messages.
        self.subscription = self.create_subscription(
            Image, 
            'video_frames', 
            self.listener_callback, 
            10)
        self.subscription  # prevent unused variable warning
        self.tick_aspect_ratio:5

        #create a publisher
        self.publisher_ = self.create_publisher(UInt8, 'stick_detected', 10)
        timer_period = 0.1 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # Used to convert between ROS and OpenCV images
        self.br = CvBridge()
        


    def timer_callback(self):
        """
        Callback function.
        """
        self.publisher_.publish(msg)
        #self.get_logger().info('Publishing: "%s"' % msg.data)
    def listener_callback(self, data):
        """
        Callback function.
        """
        # Display the message on the console
        msg.data = 199
        #self.get_logger().info('Receiving video frame')
    
        # Convert ROS Image message to OpenCV image
        current_frame = self.br.imgmsg_to_cv2(data, desired_encoding='bgr8')
        
        hsv_frame = cv2.cvtColor(current_frame, cv2.COLOR_BGR2HSV)
        cnt = 0
        lower_black = np.array([0, 0, 0])
        upper_black = np.array([180, 255, 70])

        # 定义红色色块的HSV范围
        # lower_red1 = np.array([0, 100, 100])
        # upper_red1 = np.array([10, 255, 255])
        # lower_red2 = np.array([160, 100, 100])
        # upper_red2 = np.array([180, 255, 255])

        lower_red1 = np.array([0, 80, 80])
        upper_red1 = np.array([15, 255, 255])
        lower_red2 = np.array([155, 80, 80])
        upper_red2 = np.array([180, 255, 255])


        # 创建黑色和红色的掩膜
        mask_black = cv2.inRange(hsv_frame, lower_black, upper_black)
        mask_red1 = cv2.inRange(hsv_frame, lower_red1, upper_red1)
        mask_red2 = cv2.inRange(hsv_frame, lower_red2, upper_red2)
        mask_red = cv2.bitwise_or(mask_red1, mask_red2)

        kernel = np.ones((4, 4), np.uint8)
        mask_black = cv2.morphologyEx(mask_black, cv2.MORPH_CLOSE, kernel)
        mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_CLOSE, kernel)
        # Find contours for black areas
        contours_black, _ = cv2.findContours(mask_black, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        total_area_black = sum(cv2.contourArea(contour) for contour in contours_black if cv2.contourArea(contour) > 500)
        
        # Find contours for red areas
        contours_red, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        total_area_red = sum(cv2.contourArea(contour) for contour in contours_red if cv2.contourArea(contour) > 500)

        frame_area = current_frame.shape[0] * current_frame.shape[1]
        total_area = total_area_black + total_area_red
        coverage_ratio = total_area / frame_area

        # Set the message data based on the coverage ratio and contour areas
        # if coverage_ratio > 0.2:
        #     msg.data = 202
        cnt = 0  # 初始化 cnt

        # 初始化一个标志，表示是否已经框出一个轮廓
        found = False

        # 找到黑色色块的轮廓
        contours_black, _ = cv2.findContours(mask_black, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours_black:
            if cv2.contourArea(contour) > 600:  # Filter out small areas
                x, y, w, h = cv2.boundingRect(contour)
                if h / w > stick_aspect_ratio:
                    cv2.rectangle(current_frame, (x, y), (x + w, y + h), (0, 0, 0), 2)
                    msg.data = 200
                    found = True
                    break  # Exit the loop once a valid contour is found

        # If no black contour is found, look for red contours
        if not found:
            for contour in contours_red:
                if cv2.contourArea(contour) > 600:  # Filter out small areas
                    x, y, w, h = cv2.boundingRect(contour)
                    if h / w > stick_aspect_ratio:
                        cv2.rectangle(current_frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
                        msg.data = 201
                        break  # Exit the loop once a valid contour is found
        
        # Display original image with green rectangles
        cv2.imshow("Detected Green Parts", current_frame)
        

        
        cv2.waitKey(1)
    
def main(args=None):
    # Initialize the rclpy library
    rclpy.init(args=args)
    
    # Create the node
    image_subscriber = OpencvSubscriber()
    
    # Spin the node so the callback function is called.
    rclpy.spin(image_subscriber)
    
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    image_subscriber.destroy_node()
    
    # Shutdown the ROS client library for Python
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
