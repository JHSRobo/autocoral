import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from rclpy.parameter import Parameter

class Auto(Node):
    def __init__(self):
        super().__init__('Auto')
        self.cam_subscriber = self.create_subscription(Bool, 'reached', self.callback2, 10)
        self.sensor_subscriber = self.create_subscription(Float32, 'depth_sensor', self.callback, 10)
        self.vector = self.create_publisher(Twist, 'autovector', 10)
        self.logger = self.get_logger()
        self.box = False
        self.check = False
        self.declare_parameter('autonomous_task', self.boolean)
        #self.create_timer(0.1, self.testing)
#    def testing(self):
#        self.boolean = self.get_parameter('autonomous_task').value
#        if self.boolean == True:
#            self.logger.info("auto activated")
    def callback(self, msgs):
        self.logger.info(str(msgs.data))
        self.boolean = self.get_parameter('autonomous_task').value
        if self.boolean == True:
            vector = Twist()
            if msgs.data < 10 and self.check == False:
                vector.linear.x = 0
                vector.linear.y = 0
                vector.linear.z = 0.5
                vector.angular.x = 0
                vector.angular.z = 0
            else:
                self.check = True
                if self.box == False:
                    vector.linear.x = 0.5
                    vector.linear.y = 0
                    vector.linear.z = 0
                    vector.angular.x = 0
                    vector.angular.z = 0
                elif self.box == True and msgs.data > 11:
                    vector.linear.x = 0
                    vector.linear.y = 0
                    vector.linear.z = -0.5
                    vector.angular.x = 0
                    vector.angular.z = 0
                else:
                    temp = Parameter(
                    'autonomous_task',
                    rclpy.Parameter.Type.BOOL,
                    False)
                    new_parameter = [temp]
                    self.set_parameters(new_parameter)
            self.vector.publish(vector)
    def callback2(self, msgs):
        self.box = msgs.data
        self.logger.info(self.box)
        


def main(args=None):
    rclpy.init(args=args)
    auto = Auto()
    rclpy.spin(auto)
    auto.destroy_node()
    rclpy.shutdown()



if __name__ == 'main':
    main()
