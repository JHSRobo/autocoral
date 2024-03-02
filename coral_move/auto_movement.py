import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

# webcam = cv2.VideoCapture(0)

class Auto(Node):
   def __init__(self):
       super().__init__("auto_movement")
      
       self.logger = self.get_logger()
       self.logger.info("Auto Movement Node Online")
      
       self.bridge = CvBridge()
       self.logger.info("CVBridge created")
       # self.image = None
      
       self.status = self.create_publisher(Bool, 'reached', 10)
       self.logger.info("publisher 'reached' created")
       self.camera_subscriber = self.create_subscription(Image, "camera_feed", self.img_callback, 10)
       self.logger.info("'camera_feed' subscriber created")
       # self.procedure()

   def img_callback(self, msg):
       # self.image = msg.data
       msg2 = Bool()
       # img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
       # self.get_logger().info('Publishing: "%s"' % msg.data)

       imageFrame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
       h, w, c = imageFrame.shape
       frameHeight = int(w)
       # frameHeight = int(imageFrame.get(cv2.CAP_PROP_FRAME_HEIGHT))
       # self.logger.info("imageFrame assigned")

       hsvFrame = cv2.cvtColor(imageFrame, cv2.COLOR_BGR2HSV)
        
       lower_limit = np.array([136, 87, 111], np.uint8)
       upper_limit = np.array([180, 255, 255], np.uint8)
        
       red_mask = cv2.inRange(hsvFrame, lower_limit, upper_limit)
       kernel = np.ones((5, 5), "uint8")
       red_mask = cv2.dilate(red_mask, kernel)
        
       res_red = cv2.bitwise_and(imageFrame, imageFrame, mask = red_mask)
       contours, hierarchy = cv2.findContours(red_mask,cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

       for pic, contour in enumerate(contours):
           area = cv2.contourArea(contour)
           if(area > 300):
            x, y, w, h = cv2.boundingRect(contour)
            imageFrame = cv2.rectangle(imageFrame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.putText(imageFrame, "Target Area, " + str(x) + ", " + str(y), (x, y), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0))
            if (y > frameHeight / 2 - h/2):
                msg2.data = True
            else:
                msg2.data = False

       self.status.publish(msg2)
       if msg2.data is not False:
        self.get_logger().info('Publishing: "%s"' % msg2.data)

def main():
   rclpy.init()
   auto_movement = Auto()
   rclpy.spin(auto_movement)
  
   auto_movement.destroy_node()
   rclpy.shutdown()


if __name__ == '__main__':
   main()
