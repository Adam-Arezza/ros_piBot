import rclpy
from rclpy.node import Node
import math
from std_msgs.msg import Int32
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Float32
import time

class WheelSpeedNode(Node):
    def __init__(self):
        super().__init__("velocities")
        self.node_name = "velocities"

        # Parameters
        self.declare_parameter("wheel_radius")
        self.wheel_radius = self.get_parameter("wheel_radius").get_parameter_value().double_value

        # Subscribers and Publishers
        self.cmd_wheel_vels = self.create_subscription(Float32MultiArray, "/target_velocities", self.cmd_dirs, qos_profile=10)
        self.rpm_subscriber = self.create_subscription(Float32MultiArray, "/rpm", self.compute_vels, qos_profile=10)
        self.linear_vel_pub = self.create_publisher(Float32MultiArray, "/vels", qos_profile=10)

        # Variables
        self.right_cmd_direction = 0
        self.left_cmd_direction = 0

        # Initialization message
        self.get_logger().info(f'{self.node_name} is now online.')
    
    def cmd_dirs(self, vels):
        if vels.data[0] > 0.0:
            self.right_cmd_direction = 1
        elif vels.data[0] == 0.0:
            self.right_cmd_direction = 0
        else:
            self.right_cmd_direction = -1
        
        if vels.data[1] > 0.0:
            self.left_cmd_direction = 1
        elif vels.data[1] == 0.0:
            self.left_cmd_direction = 0
        else:
            self.left_cmd_direction = -1

    def compute_vels(self, rpms):
        #linear velocities in m/s
        right_rpm = rpms.data[0]
        left_rpm = rpms.data[1]
        left_linear_vel = (left_rpm / 60) * 2 * math.pi * self.wheel_radius
        right_linear_vel = (right_rpm / 60) * 2 * math.pi * self.wheel_radius

        #correcting sign depending on direction of wheel rotation from commands
        if self.right_cmd_direction == -1:
            right_linear_vel = -1 * right_linear_vel
        if self.left_cmd_direction == -1:
            left_linear_vel = -1 * left_linear_vel

        wheel_linear_vels = Float32MultiArray()
        wheel_linear_vels.data = [right_linear_vel, left_linear_vel]
        self.linear_vel_pub.publish(wheel_linear_vels)


def main(args=None):
    rclpy.init(args=args)
    node = WheelSpeedNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
