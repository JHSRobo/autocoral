import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from rclpy.parameter import Parameter
from geometry_msgs.msg import Vector3
from rcl_interfaces.msg import SetParametersResult
import matplotlib as plt

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
        self.rangex = [640, 800]
        self.rangey = [325, 485]
        self.centerX = 720
        self.centerY = 405

        self.prev_error_x = 0
        self.integral_x = 0

        self.prev_error_y = 0
        self.integral_y = 0
        
        self.declare_parameter('autonomous_task', False)

        self.add_on_set_parameters_callback(self.parameter_callback)

        self.time_data = []
        self.error_x_data = []
        self.error_y_data = []
        
        self.start_time = self.get_clock().now()

    def parameter_callback(self, parameters):
        for parameter in parameters:
            if parameter.name == 'autonomous_task':
                if parameter.value:
                    self.logger.info("Autonomous task parameter updated: {}".format(parameter.value))
                else:
                    self.logger.info("Autonomous task parameter updated: {}".format(parameter.value))
        return SetParametersResult(successful=True)

    def callback(self, msgs):
        current_time = self.get_clock().now()
        elapsed_time = (current_time - self.start_time).nanoseconds / 1e9  #convert to seconds
        
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

            self.time_data.append(elapsed_time)
            self.error_x_data.append(error_x)
            self.error_y_data.append(error_y)

            if not self.box:
                vector.linear.z = 0.5
            else:
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
        self.logger.info(str(self.box))

    def callback3(self, msgs):
        self.coordinatex = msgs.x
        self.coordinatey = msgs.y

    @staticmethod
    def saturate(value, limit):
        return max(min(value, limit), -limit)
    
    def plot_data(self):
        plt.figure(figsize=(10, 8))

        #plt.subplot(1, 1, 1)
        plt.plot(self.time_data, self.error_x_data, label='Error X')
        plt.plot(self.time_data, self.error_y_data, label='Error Y')
        plt.title('Errors over Time')
        plt.xlabel('Time (s)')
        plt.ylabel('Error')
        plt.legend()

        plt.tight_layout()
        plt.show()

def main(args=None):
    rclpy.init(args=args)
    coral_move = Coral()
    rclpy.spin(coral_move)
    coral_move.plot_data()
    coral_move.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
