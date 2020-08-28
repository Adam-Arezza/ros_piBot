import serial 
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class Motor_driver(Node):
    def __init__(self):
        super().__init__("motor_driver")
        self.port = "/dev/ttyUSB1"
        self.ser = serial.Serial(self.port, 115200, timeout=1)
        self.motor_pwms = self.create_subscription(Float32MultiArray, "/motor_pwm_vals", self.send_motor_commands)

    def send_motor_commands(self, pwm_vals):
        pwmL = str(pwm_vals[0])
        pwmR = str(pwm_vals[1])
        msg = pwmL + ":" + pwmR
        self.ser.write(msg.encode('ascii'))

def main(args=None):
    rclpy.init(args=args)
    motors = Motor_driver()
    rclpy.spin(motors)
    motors.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()