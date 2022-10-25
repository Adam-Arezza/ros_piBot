import rclpy
from rclpy.node import Node
import math
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray


class DiffDriveNode(Node): 
    def __init__(self):
        super().__init__("diff_drive_controller")
        self.node_name = "diff_drive_controller"

        # Parameters
        self.declare_parameter("wheel_base")
        self.declare_parameter("wheel_radius")
        self.declare_parameter("max_linear_vel")
        self.declare_parameter("max_angular_vel")
        self.wheel_base = self.get_parameter("wheel_base").get_parameter_value().double_value
        self.wheel_radius = self.get_parameter("wheel_radius").get_parameter_value().double_value
        self.max_linear_vel = self.get_parameter("max_linear_vel").get_parameter_value().double_value
        self.max_angular_vel = self.get_parameter("max_angular_vel").get_parameter_value().double_value

        # Subcribers and Publishers
        self.teleop_cmds = self.create_subscription(Twist, "/cmd_vel", self.wheel_vel_calc, qos_profile=10)
        self.target_wheel_velocities = self.create_publisher(Float32MultiArray, "/target_velocities", qos_profile=10)
        # self.rpm_targets = self.create_publisher(Float32MultiArray, "/target_rpms", qos_profile=10)

        # Initialization message
        self.get_logger().info(f'{self.node_name} is now online.')
    
    # converts the input Twist msg into command velocities for right and left wheels in rad/s
    def wheel_vel_calc(self, cmd):
        vel = cmd.linear.x
        omega = cmd.angular.z

        # # capping the forward linear velocity
        # if vel > 0 and vel > self.max_linear_vel:
        #     vel = self.max_linear_vel
        # # capping the reverse linear velocity
        # if vel < 0 and abs(vel) > self.max_linear_vel:
        #     vel = -self.max_linear_vel
        
        # # capping the rotational velocity
        # if omega > 0 and omega > self.max_angular_vel:
        #     omega = self.max_angular_vel
        # if omega < 0 and abs(omega) > self.max_angular_vel:
        #     omega = -self.max_angular_vel

        # if vel == 0 and abs(omega) > 0:
        #     omega = 0.25 * omega 
        
        # computing right and left desired linear velocities
        # Publish target right and left wheel velocities
        vr = vel + ((omega * self.wheel_base) / 2)
        vl = vel - ((omega * self.wheel_base) / 2)
        wheel_vels = Float32MultiArray()
        wheel_vels.data = [vr, vl]
        self.target_wheel_velocities.publish(wheel_vels)

def main(args=None):
    rclpy.init(args=args)
    node = DiffDriveNode() 
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()