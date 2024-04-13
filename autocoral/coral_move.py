import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from rclpy.parameter import Parameter
from geometry_msgs.msg import Vector3


#dimensions of the camera frame: 1920 by 1080

class Auto(Node):
    def __init__(self):
        super().__init__('Auto')
        self.cam_subscriber = self.create_subscription(Bool, 'reached', self.callback2, 10)
        self.location_subscriber = self.create_subscription(Vector3, "coordinates", self.callback4, 10)
        self.sensor_subscriber = self.create_subscription(Float32, 'depth_sensor', self.callback, 10)
        self.imu_subscriber = self.create_subscription(Vector3, "orientation_sensor", self.callback3, 10)
        self.vector = self.create_publisher(Twist, 'autovector', 10)
        self.logger = self.get_logger()
        self.box = False
        self.check = False
        self.boolean = False
        self.declare_parameter('autonomous_task', self.boolean)
        self.x = 0
        self.coordinatex = 0
        self.coordinatey = 0
        self.rangex = []
        self.rangey = []
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
            vector.linear.x = 0
            vector.linear.x = 0
            vector.linear.y = 0
            vector.linear.z = 0
            vector.angular.x = 0
            vector.angular.z = 0
            if msgs.data < 9 and self.check == False:
                vector.linear.z = 0.5
            else:
                self.check = True
                if self.box == False:
                    vector.linear.x = 0.5
                    vector.angular.x = 1-self.x
                elif self.box == True and msgs.data > 11:
                    if self.coordinatex < self.rangex[0]:
                        self.linear.y = -0.3
                    if self.coordinatex > self.rangex[1]:
                        self.linear.y = 0.3
                    if self.coordinatey < self.rangey[0]:
                        self.linear.x = -0.3
                    if self.coordinatey > self.rangey[1]:
                        vector.linear.y = 0.3
                    else:
                        vector.linear.z = -0.5

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
    def callback3(self, msgs):
        self.x = msgs.x;
    def callback4(self, msgs):
        self.coordinatex = msgs.x
        self.coordinatey = msgs.y
        


def main(args=None):
    rclpy.init(args=args)
    auto = Auto()
    rclpy.spin(auto)
    auto.destroy_node()
    rclpy.shutdown()



if __name__ == 'main':
    main()
