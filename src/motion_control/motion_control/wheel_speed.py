import rclpy
from rclpy.node import Node
import math
from std_msgs.msg import Int32
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int32MultiArray
import time

class WheelSpeedNode(Node):
    def __init__(self):
        super().__init__("wheel_speed")
        self.node_name = "wheel_speed"

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
        self.vel_interval = self.create_timer(self.refresh_rate, self.get_velocities)

        # Subscribers and Publishers
        self.ticks = self.create_subscription(Int32MultiArray, "/ticks", self.set_ticks ,qos_profile=10)
        self.vel_publisher = self.create_publisher(Float32MultiArray, "/velocity", qos_profile=10)

        # Variables
        self.right_tick = 0
        self.left_tick = 0
        self.old_left = 0
        self.old_right = 0

        # Initialization message
        self.get_logger().info(f'{self.node_name} is now online.')
    
    def set_ticks(self, ticks):
        self.right_tick = ticks.data[0]
        self.left_tick = ticks.data[1]

    def get_velocities(self):
        circumference = self.wheel_radius * 2 * math.pi
        d_left_ticks = self.left_tick - self.old_left
        d_right_ticks = self.right_tick - self.old_right
        self.old_left = self.left_tick
        self.old_right = self.right_tick
        left_linear_vel = (d_left_ticks * self.meters_per_tick)  / self.refresh_rate
        right_linear_vel = (d_right_ticks * self.meters_per_tick) / self.refresh_rate
        left_angular_vel = left_linear_vel / self.wheel_radius
        right_angular_vel = right_linear_vel / self.wheel_radius
        dist_l = d_left_ticks * self.meters_per_tick
        dist_r = d_right_ticks * self.meters_per_tick
        dist = (dist_l + dist_r) / 2
        velocity = dist / self.refresh_rate
        wheel_vels = Float32MultiArray()
        wheel_vels.data = [right_angular_vel, left_angular_vel, velocity]
        self.vel_publisher.publish(wheel_vels)
        

def main(args=None):
    rclpy.init(args=args)
    node = WheelSpeedNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
