import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
import cv2
from cv_bridge import CvBridge

import numpy as np

stick_aspect_ratio = 3

class StickerCtr(Node):
    """
    Create an ImageSubscriber class, which is a subclass of the Node class.
    """
    def __init__(self):
        """
        Class constructor to set up the node
        """
        super().__init__('sitcker_ctr')
        
        # Create the subscriber. This subscriber will receive an Image from the video_frames topic. The queue size is 10 messages.
        self.subscription = self.create_subscription(
            Image, 
            'video_frames', 
            self.listener_callback, 
            10)
        self.subscription  # prevent unused variable warning

        # Create a publisher to publish a UInt8 message
        self.publisher_ = self.create_publisher(Point, 'stick_position', 10)
        
        # Used to convert between ROS and OpenCV images
        self.br = CvBridge()
    
    def listener_callback(self, data):
        """
        Callback function.
        """
        #self.get_logger().info('Receiving video frame')
    
        # Convert ROS Image message to OpenCV image
        current_frame = self.br.imgmsg_to_cv2(data, desired_encoding='bgr8')
        msg = Point()
        msg.z = 0.0
        hsv_frame = cv2.cvtColor(current_frame, cv2.COLOR_BGR2HSV)

        # Define the HSV range for black color (assuming the stick is black)
        lower_black = np.array([0, 0, 0])
        upper_black = np.array([180, 255, 70])

        # Define the HSV range for red color (assuming the stick is red)
        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 100, 100])
        upper_red2 = np.array([180, 255, 255])


        # Create masks for black and red colors
        mask_black = cv2.inRange(hsv_frame, lower_black, upper_black)
        mask_red1 = cv2.inRange(hsv_frame, lower_red1, upper_red1)
        mask_red2 = cv2.inRange(hsv_frame, lower_red2, upper_red2)
        mask_red = cv2.bitwise_or(mask_red1, mask_red2)

        kernel = np.ones((5, 5), np.uint8)
        mask_black = cv2.morphologyEx(mask_black, cv2.MORPH_CLOSE, kernel)
        mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_CLOSE, kernel)
        # Find contours for black areas
        contours_black, _ = cv2.findContours(mask_black, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Find contours for red areas
        contours_red, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        frame_height, frame_width = current_frame.shape[:2]
        center_x = frame_width // 2
        
        found_stick = False

        # Check black sticks
        for contour in contours_black:
            if cv2.contourArea(contour) > 500:  # Filter out small areas
                x, y, w, h = cv2.boundingRect(contour)
                if h / w > stick_aspect_ratio:
                    cv2.rectangle(current_frame, (x, y), (x + w, y + h), (0, 0, 0), 2)
                    
                    # Calculate the position of the stick's center
                    stick_center_x = x + w // 2
                    stick_center_y = y+ h //2
                    
                    # Define the threshold for the central area
                    threshold = 50
                    
                    msg.x = float(stick_center_x)
                    msg.y = float(stick_center_y)

                    found_stick = True
                    break

        # Check red sticks if no black stick is found
        if not found_stick:
            for contour in contours_red:
                if cv2.contourArea(contour) > 500:  # Filter out small areas
                    x, y, w, h = cv2.boundingRect(contour)
                    if h / w > stick_aspect_ratio:
                        cv2.rectangle(current_frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
                        
                        # Calculate the position of the stick's center
                        stick_center_x = x + w // 2
                        stick_center_y = y+ h //2
                        # Define the threshold for the central area
                        threshold = 50
                        msg.x = float(stick_center_x)
                        msg.y = float(stick_center_y)

                        found_stick = True
                        break

        # Publish the message
        self.publisher_.publish(msg)
        #self.get_logger().info('Publishing: "%s"' % msg.data)
        #print(msg.x,msg.y)
        # Display the image with detected sticks
        #cv2.imshow("Detected Stick", current_frame)
        cv2.waitKey(1)

def main(args=None):
    # Initialize the rclpy library
    rclpy.init(args=args)
    
    # Create the node
    image_subscriber = StickerCtr()
    
    # Spin the node so the callback function is called.
    rclpy.spin(image_subscriber)
    
    # Destroy the node explicitly (optional - otherwise it will be done automatically when the garbage collector destroys the node object)
    image_subscriber.destroy_node()
    
    # Shutdown the ROS client library for Python
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
