import serial
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class Motor_driver(Node):
    def __init__(self):
        super().__init__("motor_driver")
        self.port = "/dev/ttyUSB1"
        # self.ser = serial.Serial(self.port, 115200, timeout=1)
        self.declare_parameter("pwm_min")
        self.declare_parameter("pwm_max")
        self.declare_parameter("vel_min")
        self.declare_parameter("vel_max")
        self.vel_max = self.get_parameter("vel_max").get_parameter_value().double_value
        self.vel_min = self.get_parameter("vel_min").get_parameter_value().double_value
        self.pwm_min = self.get_parameter("pwm_min").get_parameter_value().integer_value
        self.pwm_max = self.get_parameter("pwm_max").get_parameter_value().integer_value
        self.motor_pwms = self.create_subscription(Float32MultiArray, "/pidR_pidL", self.send_motor_commands, qos_profile=10)
    
    def scale_vals(self, in_val):
        pwm_range = self.pwm_max - self.pwm_min
        vel_range = self.vel_max - self.vel_min
        output = (((in_val - self.vel_min) * pwm_range) / vel_range) + self.pwm_min
        return output

    def send_motor_commands(self, pid_vals):
        pid_right = pid_vals.data[0]
        pid_left = pid_vals.data[1]
        pwmL = str(self.scale_vals(pid_left))
        pwmR = str(self.scale_vals(pid_right))
        msg = pwmL + ":" + pwmR
        # self.ser.write(msg.encode('ascii'))
        self.get_logger().info(msg)

def main(args=None):
    rclpy.init(args=args)
    motors = Motor_driver()
    rclpy.spin(motors)
    motors.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()