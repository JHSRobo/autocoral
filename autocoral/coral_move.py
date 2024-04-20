import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from rclpy.parameter import Parameter
from geometry_msgs.msg import Vector3

# note to self: dimensions of the camera frame: 1920 by 1080

class Coral(Node):
    def __init__(self):
        super().__init__('Coral')

        #subscriptions
        self.depth_subscriber = self.create_subscription(Float32, 'depth_sensor', self.callback, 10)
        self.stop_subscriber = self.create_subscription(Bool, 'reached', self.callback2, 10)
        self.adjustment_subscriber = self.create_subscription(Vector3, "coordinates", self.callback3, 10)

        #publisher
        self.vector = self.create_publisher(Twist, 'cmd_vel', 10)

        #other member variables
        self.logger = self.get_logger()
        
        self.box = False
        self.check = False
        
        self.coordinatex = 0
        self.coordinatey = 0
        self.rangex = [900, 1020]
        self.rangey = [500, 580]
        
        self.declare_parameter('autonomous_task', False)

        # Add callback for parameter changes
        self.add_on_set_parameters_callback(self.parameter_callback)


    def parameter_callback(self, parameters): # used to update parameter
        for parameter in parameters:
            if parameter.name == 'autonomous_task':
                if parameter.successful:
                    self.logger.info("Autonomous task parameter updated: {}".format(parameter.value))
                else:
                    self.logger.warn("Failed to update autonomous task parameter")

    
    def callback(self, msgs):
        self.logger.info(str(msgs.data))
        self.autonomous_task = self.get_parameter('autonomous_task').value
        if self.autonomous_task:
            vector = Twist()
            if self.box == False:
                    vector.linear.z = 0.5
            else:
                if msgs.data > 11:
                    if self.coordinatex < self.rangex[0]:
                        vector.linear.y = -0.3
                    if self.coordinatex > self.rangex[1]:
                        vector.linear.y = 0.3
                    if self.coordinatey < self.rangey[0]:
                        vector.linear.x = -0.3
                    if self.coordinatey > self.rangey[1]:
                        vector.linear.y = 0.3
                    else:
                        vector.linear.z = -0.5
                else:
                    temp = Parameter('autonomous_task',rclpy.Parameter.Type.BOOL,False)
                    new_parameter = [temp]
                    self.set_parameters(new_parameter)


            self.vector.publish(vector)

    # callback2, callback3, and callback 4 only update variables which are used in the primary callback function
    def callback2(self, msgs):
        self.box = msgs.data
        self.logger.info(self.box)


    def callback3(self, msgs):
        self.coordinatex = msgs.x
        self.coordinatey = msgs.y

def main(args=None):
    rclpy.init(args=args)
    coral_move = Coral()
    rclpy.spin(coral_move)
    coral_move.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
