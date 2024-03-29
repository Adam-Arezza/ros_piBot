# import FaBo9Axis_MPU9250
import FaBo9Axis_MPU9250
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Vector3
from std_msgs.msg import Header
import tf_transformations


gyro_offset_x = -1.3698619999999953
gyro_offset_y = 1.3088929999999972
gyro_offset_z = -0.9788980000000018


class IMU_driver(Node):
    def __init__(self):
        super().__init__("imu_driver")
        self.imu = FaBo9Axis_MPU9250.MPU9250()
        self.imu_pub = self.create_publisher(Imu, "/imu/data_raw", qos_profile=10)
        self.timer = self.create_timer(0.05, self.get_data)
    
    def get_data(self):
        # imu_data = Float32MultiArray()
        self.gyro_data = self.imu.readGyro()
        self.gyro_data["x"] = self.gyro_data["x"] - gyro_offset_x
        self.gyro_data["y"] = self.gyro_data["y"] - gyro_offset_y
        self.gyro_data["z"] = self.gyro_data["z"] - gyro_offset_z
        self.accel_data = self.imu.readAccel()

        imu_msg = Imu()
        # header
        imu_msg.header = Header()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = "imu_link"
        # orientation
        imu_msg.orientation = Quaternion()
        q = tf_transformations.quaternion_from_euler(self.gyro_data["x"] , self.gyro_data["y"], self.gyro_data["z"])
        imu_msg.orientation.x = 0.0
        imu_msg.orientation.y = 0.0
        imu_msg.orientation.z = 0.0
        imu_msg.orientation.w = 0.0
        

        # # imu_msg.orientation_covariance = Float64MultiArray()
        imu_msg.orientation_covariance = [-1.0, 0.0, 0.0, -1.0, -1.0, 0.0, -1.0, 0.0, -1.0]

        # angular_velocity
        imu_msg.angular_velocity = Vector3()
        imu_msg.angular_velocity.x = self.gyro_data["x"] * 0.0174533
        imu_msg.angular_velocity.y = self.gyro_data["y"] * 0.0174533
        imu_msg.angular_velocity.z = self.gyro_data["z"] * 0.0174533
        imu_msg.angular_velocity_covariance = [0.2, 0.0, 0.0, 0.0, 0.2, 0.0, 0.0, 0.0, 0.2]
        # imu_msg.angular_velocity_covariance.data = 

        # linear_acceleration
        imu_msg.linear_acceleration = Vector3()
        imu_msg.linear_acceleration.x = self.accel_data["x"]
        imu_msg.linear_acceleration.y = self.accel_data["y"]
        imu_msg.linear_acceleration.z = self.accel_data["z"] * 9.80665
        # imu_msg.linear_acceleration_covariance = Float64MultiArray()
        imu_msg.linear_acceleration_covariance = [0.2, 0.0, 0.0, 0.0, 0.2, 0.0, 0.0, 0.0, 0.2]
        self.imu_pub.publish(imu_msg)

def main(args=None):
    rclpy.init(args=args)
    imu_node = IMU_driver()
    rclpy.spin(imu_node)
    imu_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()



