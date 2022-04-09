import time
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import RPi.GPIO as GPIO

# l293d enable pins 40,38
# l293d input pins 31,33,35,37

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)
GPIO.setup(37, GPIO.OUT)
GPIO.setup(35, GPIO.OUT)
GPIO.setup(33, GPIO.OUT)
GPIO.setup(31, GPIO.OUT)
GPIO.setup(40, GPIO.OUT)
GPIO.setup(38, GPIO.OUT)

# start with motor driver disabled
GPIO.output(40, False)
GPIO.output(38, False)

class Motor_Driver(Node):
    def __init__(self):
        super().__init__("motor_driver")
        self.node_name = "motor_driver"

        self.left_duty_cycle = 0
        self.right_duty_cycle = 0

        # Publishers
        self.dc_publisher = self.create_publisher(String, "/dcL_dcR", qos_profile=10)

        # Subscribers
        self.twist_kb_subscriber = self.create_subscription(Twist, "/cmd_vel", qos_profile=10)
        #Initialization message
        self.get_logger().info(f'{self.node_name} is now online.')
    
    

def main(args=None):
    rclpy.init(args=args)
    motors = Motor_Driver()
    rclpy.spin(motors)
    motors.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()