import FaBo9Axis_MPU9250
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class IMU_node(Node):
    def __init__(self):
        super().__init__("imu_node")
        self.imu = FaBo9Axis_MPU9250.MPU9250()
        self.imu_pub = self.create_publisher(Float32MultiArray, "/imu_data", qos_profile=1)
        self.timer = self.create_timer(0.05, self.get_data)
    
    def get_data(self):
        imu_data = Float32MultiArray()
        self.gryo_data = self.imu.readGyro()
        self.accel_data = self.imu.readAccel()
        imu_data.data = [self.gryo_data, self.accel_data]
        self.imu_pub.publish(imu_data)

def main(args=None):
    rclpy.init(args=args)
    imu_node = IMU_node()
    rclpy.spin(imu_node)
    imu_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()



