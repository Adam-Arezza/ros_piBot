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
        self.pid_out = self.create_subscription(
                                                Float32MultiArray, 
                                                "/pidR_pidL", 
                                                self.send_motor_commands, 
                                                qos_profile=10)
        self.command_velocities = self.create_subscription(
                                                           Float32MultiArray, 
                                                           "/target_velocities", 
                                                           self.set_motor_commands, 
                                                           qos_profile=10)
        self.dc_publisher = self.create_publisher(String, "/dcR_dcL", qos_profile=10)

        # Variables
        self.left_duty_cycle = 0
        self.right_duty_cycle = 0
        self.p1 = GPIO.PWM(40, 50)
        self.p2 = GPIO.PWM(38, 50)
        self.p1.start(0)
        self.p2.start(0)
        self.vr = 0
        self.vl = 0

        #Initialization message
        self.get_logger().info(f'{self.node_name} is now online.')

    def wheel_velocities(self, vels):
        self.right_wheel_velocity = vels.data[0]
        self.left_wheel_velocity = vels.data[1]
    
    def set_motor_commands(self, diff_drive_vels):
        self.vr = diff_drive_vels.data[0]
        self.vl = diff_drive_vels.data[1]
    
    def scale_vals(self, pid, target):
        vel_range = self.vel_max - self.vel_min
        dc_range = self.dc_max - self.dc_min
        output = ((pid * dc_range) / vel_range) + self.dc_min

        if target < 0:
            output = -output

        # output = abs(output)

        if output > self.dc_max:
            output = self.dc_max

        if output < self.dc_min:
            output = self.dc_min

        output = int(output)
        return output

    def send_motor_commands(self, pid_vals):
        pid_right = pid_vals.data[0]
        pid_left = pid_vals.data[1]
        dcL = 0
        dcR = 0
        dcL = self.scale_vals(pid_left, self.vl)
        dcR = self.scale_vals(pid_right, self.vr)
        # self.get_logger().info(f'{dcR}:{dcL}')
        msg = f'{dcR}:{dcL}'
        dc_msg = String()
        dc_msg.data = msg
        self.dc_publisher.publish(dc_msg)
        if self.vr == 0 and self.vl == 0:
            self.stop()
        elif self.vr > 0 and self.vl > 0:
            self.forward([dcR,dcL])
        elif self.vl < 0 and self.vr < 0:
            self.reverse([dcR,dcL])
        elif self.vl < 0 and self.vr > 0:
            self.spin_right([dcR, dcL])
        elif self.vl > 0 and self.vr < 0:
            self.spin_left([dcR,dcL])
        
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
        # self.get_logger().info(f'{duty_cycles[0]}:{duty_cycles[1]}')
    
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
        self.p2.ChangeDutyCycle(0)
    
    def spin_right(self, duty_cycles):
        #right motor
        GPIO.output(40, True)
        GPIO.output(33, True)
        GPIO.output(31, False)
        self.p1.ChangeDutyCycle(0)

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