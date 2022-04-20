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
        GPIO.setup(37, GPIO.OUT) # left motor pin b
        GPIO.setup(35, GPIO.OUT) #left motor pin a
        GPIO.setup(33, GPIO.OUT) #right motor pin b
        GPIO.setup(31, GPIO.OUT) #right motor pin a
        GPIO.setup(40, GPIO.OUT) #right motor enable
        GPIO.setup(38, GPIO.OUT) #left motor enable

        # start with motor driver disabled
        GPIO.output(40, False)
        GPIO.output(38, False)

        # Parameters
        self.declare_parameter("dc_min")
        self.declare_parameter("dc_max")
        self.declare_parameter("vel_min")
        self.declare_parameter("vel_max")
        self.vel_max = self.get_parameter("vel_max").get_parameter_value().double_value
        self.vel_min = self.get_parameter("vel_min").get_parameter_value().double_value
        self.dc_min = self.get_parameter("dc_min").get_parameter_value().integer_value
        self.dc_max = self.get_parameter("dc_max").get_parameter_value().integer_value

        # Subscribers and Publishers
        self.wheel_vels = self.create_subscription(Float32MultiArray, "/velocity", self.wheel_speeds, qos_profile=10)
        self.motor_pwms = self.create_subscription(Float32MultiArray, "/pidR_pidL", self.send_motor_commands, qos_profile=10)
        self.dc_publisher = self.create_publisher(String, "/dcR_dcL", qos_profile=10)

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

    def wheel_speeds(self, vels):
        self.right_wheel = vels.data[3]
        self.left_wheel = vels.data[4]
    
    def scale_vals(self, pid, wheel_speed):
        vel_range = self.vel_max - self.vel_min
        dc_range = self.dc_max - self.dc_min
        m_per_sec = pid + wheel_speed
        
        if pid == 0:
            m_per_sec = 0
        output = 0

        if m_per_sec < 0:
            rps = -1 * m_per_sec
            new_rps = (((rps - 0) * dc_range) / vel_range) + self.dc_min
            output = new_rps * -1

        if m_per_sec > 0:
            new_rps = (((m_per_sec - 0) * dc_range) / vel_range) + self.dc_min
            output = new_rps

        if m_per_sec == 0:
            output = 0

        if output > self.dc_max:
            output = self.dc_max

        if output < self.dc_min:
            output = self.dc_min

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
        dcL = self.scale_vals(pid_left, self.left_wheel)
        dcR = self.scale_vals(pid_right, self.right_wheel)
        if pid_left == 0 and pid_right == 0:
            self.stop()
        elif pid_left > 0 and pid_right > 0:
            self.forward([dcR,dcL])
        elif pid_left < 0 and pid_right < 0:
            self.reverse([dcR,dcL])
        elif pid_left < 0 and pid_right > 0:
            self.spin_right([dcR, dcL])
        elif pid_left > 0 and pid_right < 0:
            # bandaid fix for dc < 0
            # if dcL < 0:
            #     dcL = 0
            # if dcR < 0:
            #     dcR = 0
            self.spin_left([dcR,dcL])
        msg = f'{dcR}:{dcL}'
        dc_msg = String()
        dc_msg.data = msg
        self.dc_publisher.publish(dc_msg)
    
    def forward(self, duty_cycles):
        #right motor
        GPIO.output(40, True)
        GPIO.output(33, False)
        GPIO.output(31, True)
        self.p1.ChangeDutyCycle(duty_cycles[0])

        #left motor
        GPIO.output(37, False)
        GPIO.output(35, True)
        GPIO.output(38, True)
        self.p2.ChangeDutyCycle(duty_cycles[1])
    
    def reverse(self, duty_cycles):
        #right motor
        GPIO.output(40, True)
        GPIO.output(33, True)
        GPIO.output(31, False)
        self.p1.ChangeDutyCycle(duty_cycles[0])

        #left motor
        GPIO.output(37, True)
        GPIO.output(35, False)
        GPIO.output(38, True)
        self.p2.ChangeDutyCycle(duty_cycles[1])

    def spin_left(self, duty_cycles):
        #right motor
        GPIO.output(40, True)
        GPIO.output(33, False)
        GPIO.output(31, True)
        self.p1.ChangeDutyCycle(duty_cycles[0])

        #left motor
        GPIO.output(37, True)
        GPIO.output(35, False)
        GPIO.output(38, True)
        self.p2.ChangeDutyCycle(duty_cycles[1])
    
    def spin_right(self, duty_cycles):
        #right motor
        GPIO.output(40, True)
        GPIO.output(33, True)
        GPIO.output(31, False)
        self.p1.ChangeDutyCycle(duty_cycles[0])

        #left motor
        GPIO.output(37, False)
        GPIO.output(35, True)
        GPIO.output(38, True)
        self.p2.ChangeDutyCycle(duty_cycles[1])
    
    def stop(self):
        #right motor
        GPIO.output(40, False)
        GPIO.output(33, False)
        GPIO.output(31, False)
        self.p1.ChangeDutyCycle(0)

        #left motor
        GPIO.output(37, False)
        GPIO.output(35, False)
        GPIO.output(38, False)
        self.p2.ChangeDutyCycle(0)

def main(args=None):
    rclpy.init(args=args)
    motors = Motor_driver()
    rclpy.spin(motors)
    motors.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()