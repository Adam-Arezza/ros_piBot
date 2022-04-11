import time
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String
import RPi.GPIO as GPIO

class Motor_driver(Node):
    def __init__(self):
        super().__init__("motor_driver")
        self.node_name = "motor_driver"

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

        # Parameters
        self.declare_parameter("dc_min")
        self.declare_parameter("dc_max")
        self.declare_parameter("rad_min")
        self.declare_parameter("rad_max")
        self.rad_max = self.get_parameter("rad_max").get_parameter_value().double_value
        self.rad_min = self.get_parameter("rad_min").get_parameter_value().double_value
        self.dc_min = self.get_parameter("dc_min").get_parameter_value().integer_value
        self.dc_max = self.get_parameter("dc_max").get_parameter_value().integer_value

        # Subscribers and Publishers
        self.wheel_vels = self.create_subscription(Float32MultiArray, "/velocity", self.wheel_speeds, qos_profile=10)
        self.motor_pwms = self.create_subscription(Float32MultiArray, "/pidR_pidL", self.send_motor_commands, qos_profile=10)
        self.dc_publisher = self.create_publisher(String, "/dcL_dcR", qos_profile=10)

        # Variables
        self.right_wheel = 0.0
        self.left_wheel = 0.0
        self.left_duty_cycle = 0
        self.right_duty_cycle = 0
        self.p1 = GPIO.PWM(40, 100)
        self.p2 = GPIO.PWM(38, 100)
        self.p1.start(0)
        self.p2.start(0)

        #Initialization message
        self.get_logger().info(f'{self.node_name} is now online.')

    def wheel_speeds(self, speeds):
        self.right_wheel = speeds.data[0]
        self.left_wheel = speeds.data[1]
    
    def scale_vals(self, pid, wheel_speed):
        rad_range = self.rad_max - self.rad_min
        dc_range = self.dc_max - self.dc_min
        rad_per_sec = pid + wheel_speed
        
        if pid == 0:
            rad_per_sec = 0
        output = 0

        if rad_per_sec < 0:
            rps = -1 * rad_per_sec
            new_rps = (((rps - 0) * dc_range) / rad_range) + self.dc_min
            output = new_rps * -1

        if rad_per_sec > 0:
            new_rps = (((rad_per_sec - 0) * dc_range) / rad_range) + self.dc_min
            output = new_rps

        if rad_per_sec == 0:
            output = 0

        if output > self.dc_max:
            output = self.dc_max

        if output < -self.dc_max:
            output = -self.dc_max

        # if output > 0 and output < self.pwm_min:
        #     output = self.pwm_min
        # if output < 0 and output > -self.pwm_min:
        #     output = -self.pwm_min
        output = int(output)
        return output

    def send_motor_commands(self, pid_vals):
        pid_right = pid_vals.data[0]
        pid_left = pid_vals.data[1]
        dcL = 0
        dcR = 0
        if pid_left == 0 and pid_right == 0:
            self.stop()
        else:
            dcL = self.scale_vals(pid_left, self.right_wheel)
            dcR = self.scale_vals(pid_right, self.left_wheel)
            # bandaid fix for dc < 0
            if dcL < 0:
                dcL = 0
            if dcR < 0:
                dcR = 0
            self.move([dcR,dcL])
        msg = f'{dcR}:{dcL}'
        dc_msg = String()
        dc_msg.data = msg
        self.dc_publisher.publish(dc_msg)
    
    def move(self, duty_cycles):
        GPIO.output(40, True)
        GPIO.output(37, False)
        GPIO.output(35, True)
        GPIO.output(33, False)
        GPIO.output(31, True)
        GPIO.output(38, True)
        self.p1.ChangeDutyCycle(duty_cycles[0])
        self.p2.ChangeDutyCycle(duty_cycles[1])
    
    def stop(self):
        GPIO.output(40, False)
        GPIO.output(37, False)
        GPIO.output(35, False)
        GPIO.output(33, False)
        GPIO.output(31, False)
        GPIO.output(38, False)
        self.p1.ChangeDutyCycle(0)
        self.p2.ChangeDutyCycle(0)

def main(args=None):
    rclpy.init(args=args)
    motors = Motor_driver()
    rclpy.spin(motors)
    motors.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()