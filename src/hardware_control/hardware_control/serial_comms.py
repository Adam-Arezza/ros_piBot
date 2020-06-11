import serial
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Serial_com(Node):
    def __init__(self):
        super().__init__("serial_comms")
        self.declare_parameter("ser_msg")
        self.declare_parameter("ser_port")
        self.declare_parameter("refresh_rate")
        self.port = self.get_parameter(name="ser_port").get_parameter_value().string_value
        self.rate = self.get_parameter(name="refresh_rate").get_parameter_value().double_value()
        if not self.port:
            self.port = "/dev/ttyUSB0"
        if not self.rate:
            self.rate = 0.02
        self.ser = serial.Serial(self.port, 115200, timeout=1)
        self.publish = self.create_publisher(String, 'nano_serial_data', 20)
        self.timer = self.create_timer(self.rate, self.get_serial_data)
    
    # def msg_timer_cb(self):
    #     self.msg = self.get_parameter(name="ser_msg").get_parameter_value().string_value
    #     self.ser.write(self.msg.encode("ascii"))

    def get_serial_data(self):
        if self.ser.in_waiting > 0:
            msg = String()
            msg.data = self.ser.read_all().decode('utf8')
            self.publish.publish(msg)
            self.get_logger().info(msg.data)
            
def main(args=None):
    rclpy.init(args=args)
    my_serial = Serial_com()
    rclpy.spin(my_serial)
    my_serial.destroy_node()
    rclpy.shutdown()
if __name__ == "__main__":
    main()
