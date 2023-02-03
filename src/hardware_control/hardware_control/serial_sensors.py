import serial
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int32
from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import Imu
import tf_transformations
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Vector3
from std_msgs.msg import Header


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
        self.serial_timer = self.create_timer(self.rate, self.get_serial_data)
        self.ticks = self.create_publisher(Int32MultiArray, "/ticks", qos_profile=10)
        self.command_sub = self.create_subscription(Float32MultiArray, 
                                                  '/target_velocities', 
                                                  self.motor_commands, 
                                                  qos_profile=10)
        self.imu_pub = self.create_publisher(Imu, "/imu/data_raw", qos_profile=10)
        self.get_logger().info("Serial comms online...")

    def motor_commands(self, tv):
        self.right_target = str(tv.data[0])
        self.left_target = str(tv.data[1])
        command_message = (f'<{self.right_target}:{self.left_target}>').encode('utf-8')
        # self.get_logger().info(f'<{self.right_target}:{self.left_target}>')
        self.ser.write(command_message)
    
    def create_imu_msg(self,ax,ay,az,gx,gy,gz):
        gyro_offset_x = -1.3698619999999953
        gyro_offset_y = 1.3088929999999972
        gyro_offset_z = -0.9788980000000018
        gx_with_off = gx - gyro_offset_x
        gy_with_off = gy - gyro_offset_y
        gz_with_off = gz - gyro_offset_z
        
        imu_msg = Imu()
        # header
        imu_msg.header = Header()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = "imu_link"
        # orientation
        imu_msg.orientation = Quaternion()
        q = tf_transformations.quaternion_from_euler(gx_with_off, gy_with_off, gz_with_off)
        imu_msg.orientation.x = 0.0
        imu_msg.orientation.y = 0.0
        imu_msg.orientation.z = 0.0
        imu_msg.orientation.w = 0.0
        
        # # imu_msg.orientation_covariance = Float64MultiArray()
        imu_msg.orientation_covariance = [-1.0, 0.0, 0.0, -1.0, -1.0, 0.0, -1.0, 0.0, -1.0]

        # angular_velocity
        imu_msg.angular_velocity = Vector3()
        imu_msg.angular_velocity.x = gx_with_off* 0.0174533
        imu_msg.angular_velocity.y = gy_with_off * 0.0174533
        imu_msg.angular_velocity.z = gz_with_off * 0.0174533
        imu_msg.angular_velocity_covariance = [0.2, 0.0, 0.0, 0.0, 0.2, 0.0, 0.0, 0.0, 0.2]
        # imu_msg.angular_velocity_covariance.data = 

        # linear_acceleration
        imu_msg.linear_acceleration = Vector3()
        imu_msg.linear_acceleration.x = ax
        imu_msg.linear_acceleration.y = ay
        imu_msg.linear_acceleration.z = az * 9.80665
        # imu_msg.linear_acceleration_covariance = Float64MultiArray()
        imu_msg.linear_acceleration_covariance = [0.2, 0.0, 0.0, 0.0, 0.2, 0.0, 0.0, 0.0, 0.2]
        self.imu_pub.publish(imu_msg)

    def get_serial_data(self):
        self.ser.write(b'r')
        if self.ser.in_waiting > 0:
            try:
                # Receiveing the message and splitting components
                msg = self.ser.read_all().decode('utf8')
                # self.get_logger().info(f'{msg}')
                msg = msg.split(":")
                self.create_imu_msg(float(msg[0]),float(msg[1]),float(msg[2]),float(msg[3]),float(msg[4]),float(msg[5]))
                right_ticks_int = int(msg[6])
                left_ticks_int = int(msg[7])

                # initializing the msgs
                # distance_readings = Int32MultiArray()
                ticks = Int32MultiArray()

                # setting the data values
                ticks.data = [right_ticks_int, left_ticks_int]
                # distance_readings.data = [left_dist, middle_dist, right_dist]

                # publishing the msgs
                # self.ultrasonic.publish(distance_readings)
                self.ticks.publish(ticks)
                # self.ser.write(b'r')

            except Exception as e:
                print(e)
                # print("lol")
            # self.get_logger().info(left_ticks + ":" + right_ticks)

def main(args=None):
    rclpy.init(args=args)
    my_serial = Serial_com()
    rclpy.spin(my_serial)
    my_serial.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
