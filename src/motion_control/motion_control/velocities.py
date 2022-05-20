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
        self.declare_parameter("ticks_per_rev")
        self.declare_parameter("refresh_rate")
        self.declare_parameter("meters_per_tick")
        self.ticks_per_rev = self.get_parameter("ticks_per_rev").get_parameter_value().integer_value
        self.wheel_radius = self.get_parameter("wheel_radius").get_parameter_value().double_value
        self.refresh_rate = self.get_parameter("refresh_rate").get_parameter_value().double_value
        self.meters_per_tick = self.get_parameter("meters_per_tick").get_parameter_value().double_value

        # Timers
        self.vel_interval = self.create_timer(self.refresh_rate, self.publish_velocities)

        # Subscribers and Publishers
        self.cmd_wheel_vels = self.create_subscription(Float32MultiArray, "/vr_vl", self.wheel_cmd_vels, qos_profile=10)
        self.rpm_subscriber = self.create_subscription(Float32MultiArray, "/rpm", self.set_wheel_rpms, qos_profile=10)
        self.linear_vel_pub = self.create_publisher(Float32MultiArray, "/wheel_linear_velocity", qos_profile=10)
        self.angular_vel_pub = self.create_publisher(Float32MultiArray, "/wheel_angular_velocity", qos_profile=10)
        self.robot_velocity = self.create_publisher(Float32, "/robot_velocity", qos_profile=10)

        # Variables
        self.right_rpm = 0
        self.left_rpm = 0
        self.right_cmd_direction = 0
        self.left_cmd_direction = 0
        # self.old_time = time.time()

        # Initialization message
        self.get_logger().info(f'{self.node_name} is now online.')
    
    def set_wheel_rpms(self, rpms):
        self.right_rpm = rpms.data[0]
        self.left_rpm = rpms.data[1]
    
    def wheel_cmd_vels(self, vels):
        if vels.data[0] >= 0.0:
            self.right_cmd_direction = 1
        else:
            self.right_cmd_direction = 0
        
        if vels.data[1] >= 0.0:
            self.left_cmd_direction = 1
        else:
            self.left_cmd_direction = 0

    def publish_velocities(self):
        #linear velocities in m/s
        left_linear_vel = (self.left_rpm / 60) * 2 * math.pi * self.wheel_radius
        right_linear_vel = (self.right_rpm / 60) * 2 * math.pi * self.wheel_radius
        # angular velocities in rad/s
        left_angular_vel = left_linear_vel / self.wheel_radius
        right_angular_vel = right_linear_vel / self.wheel_radius

        #correcting sign depending on direction of wheel rotation
        if self.right_cmd_direction == 0:
            right_angular_vel = -1 * right_angular_vel
            right_linear_vel = -1 * right_linear_vel
        if self.left_cmd_direction == 0:
            left_angular_vel = -1 * left_angular_vel
            left_linear_vel = -1 * left_linear_vel

        robot_velocity = (right_linear_vel + left_linear_vel) / 2

        wheel_linear_vels = Float32MultiArray()
        wheel_linear_vels.data = [right_linear_vel, left_linear_vel]
        self.linear_vel_pub.publish(wheel_linear_vels)

        wheel_angular_vels = Float32MultiArray()
        wheel_angular_vels.data = [right_angular_vel, left_angular_vel]
        self.angular_vel_pub.publish(wheel_angular_vels)

        robot_vel = Float32()
        robot_vel.data = robot_velocity
        self.robot_velocity.publish(robot_vel)
        

def main(args=None):
    rclpy.init(args=args)
    node = WheelSpeedNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
