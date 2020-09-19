import rclpy
from rclpy.node import Node
import math
from std_msgs.msg import Int32
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int32MultiArray

class WheelSpeedNode(Node):
    def __init__(self):
        super().__init__("wheel_speed")
        self.declare_parameter("wheel_radius")
        self.declare_parameter("ticks_per_rev")
        self.declare_parameter("refresh_rate")
        self.ticks_per_rev = self.get_parameter("ticks_per_rev").get_parameter_value().integer_value
        self.wheel_radius = self.get_parameter("wheel_radius").get_parameter_value().double_value
        self.refresh_rate = self.get_parameter("refresh_rate").get_parameter_value().double_value
        self.interval = self.create_timer(self.refresh_rate, self.get_velocities)
        self.ticks = self.create_subscription(Int32MultiArray, "/ticks", self.set_ticks ,qos_profile=10)
        self.vel_publisher = self.create_publisher(Float32MultiArray, "/velR_velL", qos_profile=10)
        self.right_tick = 0
        self.left_tick = 0
        self.old_left = 0
        self.old_right = 0
        self.get_logger().info("Wheel speed node is running...")

    def set_ticks(self, ticks):
        self.right_tick = ticks.data[0]
        self.left_tick = ticks.data[1]

    def get_velocities(self):
        wheel_diameter = self.wheel_radius * 2
        circumference = wheel_diameter * math.pi
        delta_l = self.left_tick - self.old_left
        delta_r = self.right_tick - self.old_right
        self.old_left = self.left_tick
        self.old_right = self.right_tick
        meters_per_tick = circumference / self.ticks_per_rev
        left_linear_vel = (delta_l * meters_per_tick)  / self.refresh_rate
        right_linear_vel = (delta_r * meters_per_tick) / self.refresh_rate
        left_angular_vel = left_linear_vel / self.wheel_radius
        right_angular_vel = right_linear_vel / self.wheel_radius
        # left_rpm = (left_angular_vel / (2 * math.pi)) * 60
        # right_rpm = (right_angular_vel / (2 * math.pi)) * 60
        wheel_vels = Float32MultiArray()
        wheel_vels.data = [right_angular_vel, left_angular_vel]
        self.vel_publisher.publish(wheel_vels)

def main(args=None):
    rclpy.init(args=args)
    node = WheelSpeedNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
