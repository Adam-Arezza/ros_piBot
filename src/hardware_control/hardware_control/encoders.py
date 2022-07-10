import RPi.GPIO as GPIO
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Float32MultiArray

class Encoders(Node):
    def __init__(self):
        super().__init__("encoders")

        # Set up GPIO pins
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BOARD)
        self.right_encoder_pin = 13
        self.left_encoder_pin = 11
        GPIO.setup(self.right_encoder_pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        GPIO.setup(self.left_encoder_pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        GPIO.add_event_detect(self.left_encoder_pin, GPIO.BOTH, callback=self.cbl, bouncetime=2)
        GPIO.add_event_detect(self.right_encoder_pin, GPIO.BOTH, callback=self.cbr, bouncetime=2)

        # Publishers/Subscribers
        self.enc_pub = self.create_publisher(Int32MultiArray, "/ticks", qos_profile=10)
        self.velocities_pub = self.create_publisher(Float32MultiArray, "/wheel/velocities", qos_profile=10)
        self.tick_timer = self.create_timer(0.005, self.pub_ticks)
        self.pub_timer = self.create_timer(0.05, self.pub_vels)

        # Variables/Constants
        self.right_count = 0
        self.left_count = 0
        self.right_old = 0
        self.left_old = 0
        self.right_rpm_old = 0
        self.left_rpm_old = 0
        self.right_velocity_old = 0
        self.left_velocity_old = 0
        self.left_direction = 0
        self.right_direction = 0
        self.COEFF = 0.45
        self.Nticks = 40
        self.METERS_PER_TICK = 0.0049873
        
        self.get_logger().info("Encoders online.")
    
    def cbr(self,channel):
        self.right_count += 1
    
    def cbl(self,channel):
        self.left_count += 1

    def pub_ticks(self):
        enc_data = Int32MultiArray()
        enc_data.data = [self.right_count, self.left_count]
        self.enc_pub.publish(enc_data)
    
    def pub_vels(self):
        right_ticks_per_second = (self.right_count - self.right_old) / 0.05
        left_ticks_per_second = (self.left_count - self.left_old) / 0.05

        right_velocity = right_ticks_per_second * self.METERS_PER_TICK
        left_velocity = left_ticks_per_second * self.METERS_PER_TICK

        right_rpm = (right_ticks_per_second / self.Nticks) * 60
        left_rpm = (left_ticks_per_second / self.Nticks) * 60

        ewma_left_rpm = ((1 - self.COEFF) * self.left_rpm_old + self.COEFF * left_rpm)
        ewma_right_rpm = ((1 - self.COEFF) * self.right_rpm_old + self.COEFF * right_rpm)

        ewma_right_velocity = ((1 - self.COEFF) * self.right_velocity_old + self.COEFF * right_velocity)
        ewma_left_velocity = ((1 - self.COEFF) * self.left_velocity_old + self.COEFF * left_velocity)

        if right_rpm <= 0:
            ewma_right_rpm = 0.0
        if left_rpm <= 0:
            ewma_left_rpm = 0.0
        
        if right_velocity <= 0:
            ewma_right_velocity = 0.0
        if left_velocity <= 0:
            ewma_left_velocity = 0.0

        # rpm_data = Float32MultiArray()
        # rpm_data.data = [ewma_right_rpm, ewma_left_rpm]
        # self.rpm_pub.publish(rpm_data)

        velocity_data = Float32MultiArray()
        velocity_data.data = [ewma_right_velocity, ewma_left_velocity]
        self.velocities_pub.publish(velocity_data)

        self.right_old = self.right_count
        self.left_old = self.left_count
        self.right_rpm_old = ewma_right_rpm
        self.left_rpm_old = ewma_left_rpm
        self.right_velocity_old = ewma_right_velocity
        self.left_velocity_old = ewma_left_velocity

def main(args=None):
    rclpy.init(args=args)
    encoder_node = Encoders()
    rclpy.spin(encoder_node)
    encoder_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()