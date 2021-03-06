import rclpy
from rclpy.node import Node
import math
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray


class DiffDriveNode(Node): 
    def __init__(self):
        super().__init__("diff_drive_controller")
        self.declare_parameter("wheel_base")
        self.declare_parameter("wheel_radius")
        self.declare_parameter("max_linear_vel")
        self.wheel_base = self.get_parameter("wheel_base").get_parameter_value().double_value
        self.wheel_radius = self.get_parameter("wheel_radius").get_parameter_value().double_value
        self.max_linear_vel = self.get_parameter("max_linear_vel").get_parameter_value().double_value
        self.teleop_cmds = self.create_subscription(Twist, "/cmd_vel", self.wheel_vel_calc, qos_profile=10)
        self.wheel_velocities = self.create_publisher(Float32MultiArray, "/vr_vl", qos_profile=10)
        self.get_logger().info("Differential drive controller node running...")
    
    def wheel_vel_calc(self, cmd):
        vel = cmd.linear.x
        omega = cmd.angular.z
        if vel > self.max_linear_vel:
            vel = self.max_linear_vel
        vr = (2 * vel + omega * self.wheel_base) / (2 * self.wheel_radius)
        vl = (2 * vel - omega * self.wheel_base) / (2 * self.wheel_radius)
        # vr = (vr / (2 * math.pi)) * 60
        # vl = (vl / (2 * math.pi)) * 60
        wheel_vels = Float32MultiArray()
        wheel_vels.data = [vr, vl]
        self.wheel_velocities.publish(wheel_vels)

def main(args=None):
    rclpy.init(args=args)
    node = DiffDriveNode() 
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()