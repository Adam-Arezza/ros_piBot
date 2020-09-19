import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int32
from std_msgs.msg import Int32MultiArray


class PoseEstimator(Node): 
    def __init__(self):
        super().__init__("pose_estimator")
        self.robot_pose = self.create_publisher(Pose2D, "robot_pose", qos_profile=10)
        self.imu_data = self.create_subscription(Float32MultiArray, "imu_data", self.set_imu_vals)
        self.ultrasonic_data = self.create_subscription(Int32MultiArray, "ultrasonic", self.set_ultrasonic_val)
        self.control_timer = self.create_timer(0.01, self.main_loop)
        self.imu_gryo_z = None
        self.left_wheel_ticks = None
        self.right_wheel_ticks = None
        self.ultrasonic_array = None
    
    # def main_loop():



def main(args=None):
    rclpy.init(args=args)
    node = PoseEstimator() 
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()