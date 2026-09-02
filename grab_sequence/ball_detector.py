import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge # a bridge to translate between ROS's 'Image' message format and OpenCV's native NumPy format
import cv2
import numpy as np # used to define the HSV color range as an array

class BallDetector(Node): # new node class that inherits from Node
    def __init__(self):
        super().__init__('ball_detector') # registers this node under the name 'ball detector'
        self.bridge = CvBridge()
        self.subscription = self.create_subscription( Image, '/zed/zed_node/rgb/image_rect_color', self.image_callback, 10) # subscribes to the camera topic

    def image_callback(self, msg): # this function runs automatically each ti,e a new 'Image' message is published on the subscribed topic
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8') # converts the ROS message into an OpenCV NumPy array with 'bgr8' encoding
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV) # switches color space from BGR to HSV
        lower_yellow = np.array([20, 100, 100]) # lower threshold of what is considered yellow
        upper_yellow = np.array([35, 255, 255]) # upper threshold of what is considered yellow
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow) # creates a binary mask of the yellow area
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE) # draws outlines of the yellow in the frame
        if contours:
            largest = max(contours, key=cv2.contourArea) # chooses the largest contour
            M = cv2.moments(largest)
            if M['m00'] > 0:
                u = int(M['m10'] / M['m00'])
                v = int(M['m01'] / M['m00'])
                self.get_logger().info(f'Ball detected at pixel ({u}, {v})', throttle_duration_sec=1.0)

def main():
        rclpy.init()
        node = BallDetector()
        rclpy.spin(node)

if __name__ == '__main__':
    main()