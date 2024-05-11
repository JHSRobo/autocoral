import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from rclpy.parameter import Parameter
from geometry_msgs.msg import Vector3

Kp_x = 0.5  # proportional gain for x
Ki_x = 0.01  # integral gain for x
Kd_x = 0.1  # derivative gain for x

Kp_y = 0.5  # proportional gain for y
Ki_y = 0.01  # integral gain for y
Kd_y = 0.1  # derivative gain for y

class Coral(Node):
    def __init__(self):
        super().__init__('Coral')

        self.depth_subscriber = self.create_subscription(Float32, 'depth_sensor', self.callback, 10)
        self.stop_subscriber = self.create_subscription(Bool, 'reached', self.callback2, 10)
        self.adjustment_subscriber = self.create_subscription(Vector3, "coordinates", self.callback3, 10)

        self.vector = self.create_publisher(Twist, 'cmd_vel', 10)

        self.logger = self.get_logger()
        
        self.box = False
        
        self.coordinatex = 0
        self.coordinatey = 0
        self.rangex = [640, 800] #880, 1040
        self.rangey = [325, 485] #480, 600
        self.centerX = 720 #960
        self.centerY = 405 #540

        self.prev_error_x = 0
        self.integral_x = 0

        self.prev_error_y = 0
        self.integral_y = 0
        
        self.declare_parameter('autonomous_task', False)

        self.add_on_set_parameters_callback(self.parameter_callback)

    def parameter_callback(self, parameters):
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
            error_x = self.centerX - self.coordinatex
            error_y = self.centerY - self.coordinatey

            self.integral_x += error_x
            derivative_x = error_x - self.prev_error_x
            control_x = Kp_x * error_x + Ki_x * self.integral_x + Kd_x * derivative_x

            self.integral_y += error_y
            derivative_y = error_y - self.prev_error_y
            control_y = Kp_y * error_y + Ki_y * self.integral_y + Kd_y * derivative_y
	
            control_x = self.saturate(control_x, 1.0)
            control_y = self.saturate(control_y, 1.0)

            if self.box == False:
                    vector.linear.z = 0.5
            elif self.box == True:
                if (self.coordinatex < self.rangex[0] or self.coordinatex > self.rangex[1]) or (self.coordinatey < self.rangey[0] or self.coordinatey > self.rangey[1]):
                    vector.linear.y = 0.3 + control_x
                    vector.linear.x = 0.3 + control_y
                else:
                    if msgs.data < 11:
                        vector.linear.z = -0.5

            self.prev_error_x = error_x
            self.prev_error_y = error_y

            self.vector.publish(vector)

    def callback2(self, msgs):
        self.box = msgs.data
        self.logger.info(self.box)

    def callback3(self, msgs):
        self.coordinatex = msgs.x
        self.coordinatey = msgs.y

    def saturate(value, limit):
        return max(min(value, limit), -limit)

def main(args=None):
    rclpy.init(args=args)
    coral_move = Coral()
    rclpy.spin(coral_move)
    coral_move.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
