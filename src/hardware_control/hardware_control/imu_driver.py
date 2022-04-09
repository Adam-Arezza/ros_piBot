# import FaBo9Axis_MPU9250
import FaBo9Axis_MPU9250
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


gyro_offset_x = -1.872561000000002
gyro_offset_y = 1.4569519999999994
gyro_offset_z = -0.6041550000000008


class IMU_driver(Node):
    def __init__(self):
        super().__init__("imu_driver")
        self.imu = FaBo9Axis_MPU9250.MPU9250()
        self.imu_pub = self.create_publisher(Float32MultiArray, "/imu_data", qos_profile=10)
        self.timer = self.create_timer(0.05, self.get_data)
    
    def get_data(self):
        imu_data = Float32MultiArray()
        self.gyro_data = self.imu.readGyro()
        self.gyro_data["x"] = self.gyro_data["x"] - gyro_offset_x
        self.gyro_data["y"] = self.gyro_data["y"] - gyro_offset_y
        self.gyro_data["z"] = self.gyro_data["z"] - gyro_offset_z
        self.accel_data = self.imu.readAccel()
        imu_data.data = [self.gyro_data["x"], self.gyro_data["y"], self.gyro_data["z"], self.accel_data["x"], self.accel_data["y"], self.accel_data["z"]]
        self.imu_pub.publish(imu_data)

def main(args=None):
    rclpy.init(args=args)
    imu_node = IMU_driver()
    rclpy.spin(imu_node)
    imu_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()



