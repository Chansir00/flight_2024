import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import UInt8
import cv2
from cv_bridge import CvBridge

import numpy as np


msg = UInt8()
class DownImageSubscriber(Node):
    """
    Create an ImageSubscriber class, which is a subclass of the Node class.
    """
    def __init__(self):
        """
        Class constructor to set up the node
        """
        # Initiate the Node class's constructor and give it a name
        super().__init__('down_cam_subscriber')
        
        # Create the subscriber. This subscriber will receive an Image
        # from the video_frames topic. The queue size is 10 messages.
        self.subscription = self.create_subscription(
            Image, 
            'down_video_frames', 
            self.listener_callback, 
            10)
        self.subscription  # prevent unused variable warning
        
        # Create a publisher to publish a UInt8 message
        self.publisher_ = self.create_publisher(UInt8, 'circle_detected', 10)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        # Used to convert between ROS and OpenCV images
        self.br = CvBridge()


    def timer_callback(self):
        """
        Callback function.
        """
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
    
    def listener_callback(self, data):
        """
        Callback function.
        """
        # Display the message on the console
        self.get_logger().info('Receiving video frame')
    
        # Convert ROS Image message to OpenCV image
        current_frame = self.br.imgmsg_to_cv2(data)
        
        # Convert to grayscale
        gray_frame = cv2.cvtColor(current_frame, cv2.COLOR_BGR2GRAY)
        gray_frame = cv2.medianBlur(gray_frame, 5)
        
        # Detect circles using HoughCircles
        circles = cv2.HoughCircles(gray_frame, cv2.HOUGH_GRADIENT, dp=1.2, minDist=30,
                                   param1=50, param2=70, minRadius=25, maxRadius=500)
        

        if circles is not None:
            circles = np.uint16(np.around(circles))
            for i in circles[0, :]:
                # Draw the outer circle
                cv2.circle(current_frame, (i[0], i[1]), i[2], (0, 255, 0), 2)
                # Draw the center of the circle
                cv2.circle(current_frame, (i[0], i[1]), 2, (0, 0, 255), 3)
            
            # Set message data to 1 if circles are detected
            msg.data = 200
        else:
            # Set message data to 0 if no circles are detected
            msg.data = 199
        
        cv2.imshow("camera", current_frame)
        
        cv2.waitKey(1)
        

def main(args=None):
    # Initialize the rclpy library
    rclpy.init(args=args)
    
    # Create the node
    image_subscriber = DownImageSubscriber()
    
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
