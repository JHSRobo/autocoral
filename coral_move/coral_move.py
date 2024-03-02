import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from std_msgs.msg import Float32

class Auto(Node):
    def __init__(self):
        super().__init__('Auto')
        #self.cam_subscriber = self.create_subscription(Bool, 'reached', self.callback2, 10)
        self.sensor_subscriber = self.create_subscription(Float32, 'depth_sensor', self.callback, 10)
        self.vector = self.create_publisher(Twist, 'autovector', 10)
        self.logger = self.get_logger()
        self.box = False
        self.check = False
    def callback(self, msgs):
        self.logger.info(str(msgs.data))
        vector = Twist()
        if msgs.data < 10 and self.check == False:
            vector.linear.x = 0
            vector.linear.y = 0
            vector.linear.z = -1
            vector.angular.x = 0
            vector.angular.z = 0
        else:
            self.check = True
            if self.box == False:
                vector.linear.x = 1
                vector.linear.y = 0
                vector.linear.z = 0
                vector.angular.x = 0
                vector.angular.z = 0
            elif self.box == True and msgs.data > 11:
                vector.linear.x = 0
                vector.linear.y = 0
                vector.linear.z = 1
                vector.angular.x = 0
                vector.angular.z = 0

        self.vector.publish(vector)
    #def callback2(self, msg):
    #    if msg == True:
    #        self.box = msg
        


def main(args=None):
    rclpy.init(args=args)
    auto = Auto()
    rclpy.spin(auto)
    auto.destroy_node()
    rclpy.shutdown()



if __name__ == 'main':
    main()
