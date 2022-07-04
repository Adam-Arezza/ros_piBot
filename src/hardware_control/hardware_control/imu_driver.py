# import FaBo9Axis_MPU9250
import FaBo9Axis_MPU9250
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Header
import tf_transformations


gyro_offset_x = -1.872561000000002
gyro_offset_y = 1.4569519999999994
gyro_offset_z = -0.6041550000000008


class IMU_driver(Node):
    def __init__(self):
        super().__init__("imu_driver")
        self.imu = FaBo9Axis_MPU9250.MPU9250()
        self.imu_pub = self.create_publisher(Imu, "/imu/data_raw", qos_profile=10)
        self.timer = self.create_timer(0.05, self.get_data)
    
    def get_data(self):
        imu_data = Float32MultiArray()
        self.gyro_data = self.imu.readGyro()
        self.gyro_data["x"] = self.gyro_data["x"] - gyro_offset_x
        self.gyro_data["y"] = self.gyro_data["y"] - gyro_offset_y
        self.gyro_data["z"] = self.gyro_data["z"] - gyro_offset_z
        self.accel_data = self.imu.readAccel()
        # imu_data.data = [self.gyro_data["x"], self.gyro_data["y"], self.gyro_data["z"], self.accel_data["x"], self.accel_data["y"], self.accel_data["z"]]
        # self.imu_pub.publish(imu_data)

        imu_msg = Imu()
        # header
        imu_msg.header = Header()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = "imu_link"
        # orientation
        imu_msg.orientation = Quaternion()
        q = tf_transformations.quaternion_from_euler(self.gyro_data["x"] , self.gyro_data["y"], self.gyro_data["z"])
        imu_msg.orientation.x = q[0]
        imu_msg.orientation.y = q[1]
        imu_msg.orientation.z = q[2]
        imu_msg.orientation.w = q[3]

        # imu_msg.orientation_covariance = Float64MultiArray()
        imu_msg.orientation_covariance = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]

        # angular_velocity
        imu_msg.angular_velocity = Vector3()
        imu_msg.angular_velocity.x = self.gyro_data["x"] * 0.0174533
        imu_msg.angular_velocity.y = self.gyro_data["y"] * 0.0174533
        imu_msg.angular_velocity.z = self.gyro_data["z"] * 0.0174533
        imu_msg.angular_velocity_covariance = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        # imu_msg.angular_velocity_covariance.data = 

        # linear_velocity
        imu_msg.linear_acceleration = Vector3()
        imu_msg.linear_acceleration.x = self.accel_data["x"] * 9.80665 
        imu_msg.linear_acceleration.y = self.accel_data["y"] * 9.80665
        imu_msg.linear_acceleration.z = self.accel_data["z"] * 9.80665
        # imu_msg.linear_acceleration_covariance = Float64MultiArray()
        imu_msg.linear_acceleration_covariance = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        self.imu_pub.publish(imu_msg)

def main(args=None):
    rclpy.init(args=args)
    imu_node = IMU_driver()
    rclpy.spin(imu_node)
    imu_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()



