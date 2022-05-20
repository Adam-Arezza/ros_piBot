import RPi.GPIO as GPIO
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Float32MultiArray

class Encoders(Node):
    def __init__(self):
        super().__init__("encoders")
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BOARD)
        self.right_encoder_pin = 13
        self.left_encoder_pin = 11
        GPIO.setup(self.right_encoder_pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        GPIO.setup(self.left_encoder_pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        GPIO.add_event_detect(self.left_encoder_pin, GPIO.BOTH, callback=self.cbr, bouncetime=2)
        GPIO.add_event_detect(self.right_encoder_pin, GPIO.BOTH, callback=self.cbl, bouncetime=2)
        self.right_count = 0
        self.left_count = 0
        self.right_old = 0
        self.left_old = 0
        self.Nticks = 40
        self.enc_pub = self.create_publisher(Int32MultiArray, "/ticks", qos_profile=10)
        self.rpm_pub = self.create_publisher(Float32MultiArray, "/rpm", qos_profile=10)
        self.tick_timer = self.create_timer(0.002, self.pub_ticks)
        self.rpm_timer = self.create_timer(0.1, self.pub_rpm)
        self.get_logger().info("Encoders online.")
    
    def cbr(self,channel):
        self.right_count += 1
    
    def cbl(self,channel):
        self.left_count += 1

    def pub_ticks(self):
        enc_data = Int32MultiArray()
        enc_data.data = [self.right_count, self.left_count]
        self.enc_pub.publish(enc_data)
    
    def pub_rpm(self):
        right_ticks_per_second = (self.right_count - self.right_old) / 0.1
        left_ticks_per_second = (self.left_count - self.left_old) / 0.1
        right_rpm = (right_ticks_per_second / self.Nticks) * 60
        left_rpm = (left_ticks_per_second / self.Nticks) * 60
        rpm_data = Float32MultiArray()
        rpm_data.data = [right_rpm, left_rpm]
        self.rpm_pub.publish(rpm_data)
        self.right_old = self.right_count
        self.left_old = self.left_count

def main(args=None):
    rclpy.init(args=args)
    encoder_node = Encoders()
    rclpy.spin(encoder_node)
    encoder_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()