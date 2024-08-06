import rclpy

from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

class DownImagePublisher(Node):

    def __init__(self):
        super().__init__('down_image_publisher')
        self.publisher_ = self.create_publisher(Image, 'down_video_frames', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.cap = cv2.VideoCapture(1, cv2.CAP_V4L2)
        

        # for i in range(3):
        #     try:
        #         cap = cv2.VideoCapture(i, cv2.CAP_V4L2)
        #         if cap.isOpened():
        #             self.cap = cap
        #             break
        #     except Exception as e:
        #         print(f"Error opening camera {i}: {e}")
        #         continue
        #self.cap.set(6,cv2.VideoWriter.fourcc('M','J','P','G'))
        # width = 640
        # height = 480
        # self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        # self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.bridge = CvBridge()

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            self.publisher_.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    image_publisher = DownImagePublisher()

    try:
        rclpy.spin(image_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        image_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
