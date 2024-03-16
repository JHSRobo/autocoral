import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from core.msg import RectDimensions

class Auto(Node):
    def __init__(self):
        super().__init__("target_detection")
      
        self.logger = self.get_logger()
        self.logger.info("Target Detection Node Online")
      
        self.bridge = CvBridge()
      
        self.status = self.create_publisher(Bool, 'reached', 10)
        self.red_outline = self.create_publisher(RectDimensions, 'parameters', 10)
        self.camera_subscriber = self.create_subscription(Image, "camera_feed", self.img_callback, 10)

    def img_callback(self, msg):
        my_param = self.get_parameter('autonomous_task').get_parameter_value().bool_value
        if my_param == True:
            msg2 = Bool()
            msg3 = RectDimensions()
            contours_detected = False
    
            imageFrame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            h, w, c = imageFrame.shape
            frameHeight = int(w)
    
            hsvFrame = cv2.cvtColor(imageFrame, cv2.COLOR_BGR2HSV)
            
            lower_limit = np.array([136, 87, 111], np.uint8)
            upper_limit = np.array([180, 255, 255], np.uint8)
            
            red_mask = cv2.inRange(hsvFrame, lower_limit, upper_limit)
            kernel = np.ones((5, 5), "uint8")
            red_mask = cv2.dilate(red_mask, kernel)
            
            res_red = cv2.bitwise_and(imageFrame, imageFrame, mask = red_mask)
            contours, hierarchy = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
            for pic, contour in enumerate(contours):
                area = cv2.contourArea(contour)
                if(area > 2000):
                    x, y, w, h = cv2.boundingRect(contour)
                    msg3.x = x
                    msg3.y = y
                    msg3.w = w
                    msg3.h = h
                    self.red_outline.publish(msg3)
                    contours_detected = True
                    
                    if (y > frameHeight / 2 - h/2):
                        msg2.data = True
                    else:
                        msg2.data = False
    
            if not contours_detected:
                # Publish default RectDimensions message with all values set to 0
                self.red_outline.publish(RectDimensions())
    
            self.status.publish(msg2)
            if msg2.data is not False:
                self.get_logger().info('Publishing: "%s"' % msg2.data)

def main():
    rclpy.init()
    target_detection = Auto()
    rclpy.spin(target_detection)
    target_detection.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
