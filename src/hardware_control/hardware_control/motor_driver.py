import serial
import time
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String

class Motor_driver(Node):
    def __init__(self):
        super().__init__("motor_driver")
        self.port = "/dev/ttyUSB1"
        self.ser = serial.Serial(self.port, 115200, timeout=1)
        self.declare_parameter("pwm_min")
        self.declare_parameter("pwm_max")
        self.declare_parameter("rad_min")
        self.declare_parameter("rad_max")
        self.rad_max = self.get_parameter("rad_max").get_parameter_value().double_value
        self.rad_min = self.get_parameter("rad_min").get_parameter_value().double_value
        self.pwm_min = self.get_parameter("pwm_min").get_parameter_value().integer_value
        self.pwm_max = self.get_parameter("pwm_max").get_parameter_value().integer_value
        self.motor_pwms = self.create_subscription(Float32MultiArray, "/pidR_pidL", self.send_motor_commands, qos_profile=10)
        self.pwm_publisher = self.create_publisher(String, "pwmL_pwmR", qos_profile=10)
    
    def scale_vals(self, in_val):
        pwm_range = self.pwm_max - self.pwm_min
        rad_range = self.rad_max - self.rad_min
        output = (((in_val - self.rad_min) * pwm_range) / rad_range) + self.pwm_min
        if output > self.pwm_max:
            output = self.pwm_max
        elif output <= self.pwm_min:
            output = 0
        output = int(output)
        return output

    def send_motor_commands(self, pid_vals):
        pid_right = pid_vals.data[0]
        pid_left = pid_vals.data[1]
        pwmL = str(self.scale_vals(pid_left))
        pwmR = str(self.scale_vals(pid_right))
        msg = pwmL + ":" + pwmR + "\n"
        pwm_msg = String()
        pwm_msg.data = msg
        self.ser.write(msg.encode('ascii'))
        self.pwm_publisher.publish(pwm_msg)
        # self.get_logger().info(msg)

def main(args=None):
    rclpy.init(args=args)
    motors = Motor_driver()
    rclpy.spin(motors)
    motors.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()