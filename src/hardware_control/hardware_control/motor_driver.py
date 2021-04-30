import serial
import time
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class Motor_driver(Node):
    def __init__(self):
        super().__init__("motor_driver")
        self.node_name = "motor_driver"
        self.port = "/dev/ttyUSB1"
        self.ser = serial.Serial(self.port, 115200, timeout=1)

        # Parameters
        self.declare_parameter("pwm_min")
        self.declare_parameter("pwm_max")
        self.declare_parameter("rad_min")
        self.declare_parameter("rad_max")
        self.rad_max = self.get_parameter("rad_max").get_parameter_value().double_value
        self.rad_min = self.get_parameter("rad_min").get_parameter_value().double_value
        self.pwm_min = self.get_parameter("pwm_min").get_parameter_value().integer_value
        self.pwm_max = self.get_parameter("pwm_max").get_parameter_value().integer_value

        # Subscribers and Publishers
        self.wheel_vels = self.create_subscription(Float32MultiArray, "/velocity", self.wheel_speeds, qos_profile=10)
        self.motor_pwms = self.create_subscription(Float32MultiArray, "/pidR_pidL", self.send_motor_commands, qos_profile=10)
        self.pwm_publisher = self.create_publisher(String, "/pwmL_pwmR", qos_profile=10)

        # Variables
        self.right_wheel = 0.0
        self.left_wheel = 0.0

        #Initialization message
        self.get_logger().info(f'{self.node_name} is now online.')

    def wheel_speeds(self, speeds):
        self.right_wheel = speeds.data[0]
        self.left_wheel = speeds.data[1]
    
    def scale_vals(self, pid, wheel_speed):
        rad_range = self.rad_max - self.rad_min
        pwm_range = self.pwm_max - self.pwm_min
        rad_per_sec = pid + wheel_speed
        
        if pid == 0:
            rad_per_sec = 0
        output = 0

        if rad_per_sec < 0:
            rps = -1 * rad_per_sec
            new_rps = (((rps - 0) * pwm_range) / rad_range) + self.pwm_min
            output = new_rps * -1

        if rad_per_sec > 0:
            new_rps = (((rad_per_sec - 0) * pwm_range) / rad_range) + self.pwm_min
            output = new_rps

        if rad_per_sec == 0:
            output = 0

        if output > self.pwm_max:
            output = self.pwm_max

        if output < -self.pwm_max:
            output = -self.pwm_max

        # if output > 0 and output < self.pwm_min:
        #     output = self.pwm_min
        # if output < 0 and output > -self.pwm_min:
        #     output = -self.pwm_min
        output = int(output)
        return output

    def send_motor_commands(self, pid_vals):
        pid_right = pid_vals.data[0]
        pid_left = pid_vals.data[1]
        pwmL = str(self.scale_vals(pid_left, self.right_wheel))
        pwmR = str(self.scale_vals(pid_right, self.left_wheel))
        msg = f'<{pwmR}:{pwmL}>'
        pwm_msg = String()
        pwm_msg.data = msg
        self.ser.write(msg.encode('ascii'))
        self.pwm_publisher.publish(pwm_msg)

def main(args=None):
    rclpy.init(args=args)
    motors = Motor_driver()
    rclpy.spin(motors)
    motors.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()