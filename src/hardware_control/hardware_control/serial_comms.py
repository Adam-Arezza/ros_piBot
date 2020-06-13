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
        self.declare_parameter("motor_control")
        self.declare_parameter("encoder_left")
        self.declare_parameter("encoder_right")
        self.declare_parameter("ultrasonic_array")
        self.port = self.get_parameter(
            name="ser_port").get_parameter_value().string_value
        self.rate = self.get_parameter(
            name="refresh_rate").get_parameter_value().double_value
        self.motor_control_enable = self.get_parameter(
            name="motor_control").get_parameter_value().bool_value
        if not self.port:
            self.port = "/dev/ttyUSB0"
        if not self.rate:
            self.rate = 0.02
        self.ser = serial.Serial(self.port, 115200, timeout=1)
        # self.publish = self.create_publisher(
        #     String, 'nano_serial_data', qos_profile=None)
        if self.motor_control_enable:
            self.subscribe = self.create_subscription(
                Float32MultiArray, "motor_pwm_vals", self.send_serial_data, qos_profile=None)
        self.receive_timer = self.create_timer(self.rate, self.get_serial_data)
        if self.get_parameter(name="encoder_left").get_parameter_value().string_value != None:
            self.encoder_left = self.create_publisher(
                Int32, "left_tick", qos_profile=None)
        if self.get_parameter(name="encoder_right").get_parameter_value().string_value != None:
            self.encoder_right = self.create_publisher(
                Int32, "right_tick", qos_profile=None)
        if self.get_parameter(name="ultrasonic_array").get_parameter_value().string_value != None:
            self.ultrasonic = self.create_publisher(
                Int32MultiArray, "ultrasonic", qos_profile=None)
    # def msg_timer_cb(self):
    #     self.msg = self.get_parameter(name="ser_msg").get_parameter_value().string_value
    #     self.ser.write(self.msg.encode("ascii"))

    def get_serial_data(self):
        if self.ser.in_waiting > 0:
            # msg = String()
            # msg.data = self.ser.read_all().decode('utf8')
            # msg.data = msg.data.split("-")
            # self.publish.publish(msg)
            msg = self.ser.read_all().decode('utf8')
            msg = msg.split("-")
            right_ticks = msg[0].split(":")[1]
            left_ticks = msg[1].split(":")[1]
            right_ticks_int = int(right_ticks)
            left_ticks_int = int(left_ticks)
            left_tick = Int32()
            right_tick = Int32()
            left_tick.data = left_ticks_int
            right_tick.data = right_ticks_int
            self.encoder_left.publish(left_tick)
            self.encoder_right.publish(right_tick)
            # self.get_logger().info(left_ticks + ":" + right_ticks)

    def send_serial_data(self, msg):
        self.ser.write(msg.data.encode('ascii'))
        self.ser.write(b"\n")
        self.get_logger().info(msg.data)

def main(args=None):
    rclpy.init(args=args)
    my_serial=Serial_com()
    rclpy.spin(my_serial)
    my_serial.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
