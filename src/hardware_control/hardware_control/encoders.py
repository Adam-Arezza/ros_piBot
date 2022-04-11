import RPi.GPIO as GPIO
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray

class Encoders(Node):
    def __init__(self):
        super().__init__("encoders")
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(11, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        GPIO.setup(13, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        GPIO.add_event_detect(13, GPIO.RISING, callback=self.cbr, bouncetime=10)
        GPIO.add_event_detect(11, GPIO.RISING, callback=self.cbl, bouncetime=10)
        self.right_count = 0
        self.left_count = 0
        self.enc_pub = self.create_publisher(Int32MultiArray, "/enc_counts", qos_profile=10)
        self.timer = self.create_timer(0.05, self.get_data)
        self.get_logger().info("Encoders online.")
    
    def cbr(self,channel):
        self.right_count += 1
    
    def cbl(self,channel):
        self.left_count += 1

    def get_data(self):
        enc_data = Int32MultiArray()
        enc_data.data = [self.right_count, self.left_count]
        self.enc_pub.publish(enc_data)

def main(args=None):
    rclpy.init(args=args)
    encoder_node = Encoders()
    rclpy.spin(encoder_node)
    encoder_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()