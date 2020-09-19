import serial
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int32
from std_msgs.msg import Int32MultiArray

class Serial_com(Node):
    def __init__(self):
        super().__init__("serial_comms")
        self.declare_parameter("ser_port")
        self.declare_parameter("refresh_rate")
        self.port = self.get_parameter(name="ser_port").get_parameter_value().string_value
        self.rate = self.get_parameter(name="refresh_rate").get_parameter_value().double_value
        if not self.port:
            self.port = "/dev/ttyUSB0"
        if not self.rate:
            self.rate = 0.02
        self.ser = serial.Serial(self.port, 115200, timeout=1)
        self.receive_timer = self.create_timer(self.rate, self.get_serial_data)
        self.encoder_ticks = self.create_publisher(Int32MultiArray, "ticks", qos_profile=10)
        self.ultrasonic = self.create_publisher(Int32MultiArray, "ultrasonic", qos_profile=10)
        self.get_logger().info("Serial comms online...")

    def get_serial_data(self):
        if self.ser.in_waiting > 0:
            try:
                # Receiveing the message and splitting components
                msg = self.ser.read_all().decode('utf8')
                msg = msg.split(":")
                left_dist = int(msg[0])
                middle_dist = int(msg[1])
                right_dist = int(msg[2])
                right_ticks_int = int(msg[3])
                left_ticks_int = int(msg[4])

                # initializing the msgs
                distance_readings = Int32MultiArray()
                ticks = Int32MultiArray()

                # setting the data values
                ticks.data = [right_ticks_int, left_ticks_int]
                distance_readings.data = [left_dist, middle_dist, right_dist]

                # publishing the msgs
                self.ultrasonic.publish(distance_readings)
                self.encoder_ticks.publish(ticks)

            except:
                print("lol")
            # self.get_logger().info(left_ticks + ":" + right_ticks)

def main(args=None):
    rclpy.init(args=args)
    my_serial = Serial_com()
    rclpy.spin(my_serial)
    my_serial.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
