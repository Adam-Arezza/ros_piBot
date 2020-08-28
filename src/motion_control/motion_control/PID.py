import rclpy
import math
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int32


class PID_node(Node): # MODIFY NAME
    def __init__(self):
        super().__init__("pid_node") # MODIFY NAME
        self.declare_parameter("kp")
        self.declare_parameter("ki")
        self.declare_parameter("kd")
        self.declare_parameter("wheel_radius")
        self.declare_parameter("ticks_per_rev")
        self.ticks_per_rev = self.get_parameter("ticks_per_rev").get_parameter_value()
        self.kp = self.get_parameter("kp").get_parameter_value()
        self.ki = self.get_parameter("ki").get_parameter_value()
        self.kd = self.get_parameter("kd").get_parameter_value()
        self.wheel_radius = self.get_parameter("wheel_radius").get_parameter_value()
        self.target_right_ws = 0
        self.target_left_ws = 0
        self.vel_timer = self.create_timer(0.1, self.get_velocities)
        self.target_vels = self.create_subscription(Float32MultiArray, "/vr_vl", self.set_targets)
        self.encoder_left_sub = self.create_subscription(Int32, "/left_tick", self.set_left_tick)
        self.encoder_right_sub = self.create_subscription(Int32, "/right_tick", self.set_right_tick)
        self.right_tick = 0
        self.left_tick = 0
        self.old_left = 0
        self.old_right = 0
        self.right_vel = 0
        self.left_vel = 0
        self.pid_timer = self.create_timer(0.1, self.pid_loop)
        self.get_logger().info("PID controller has started running...")
    
    def set_targets(self, targets):
        self.target_right_ws = targets.data[0]
        self.target_left_ws = targets.data[1]
    
    def set_left_tick(self, ticks):
        self.old_left = self.left_tick
        self.left_tick = ticks.data
    
    def set_right_tick(self, ticks):
        self.old_right = self.right_tick
        self.right_tick = ticks.data
    
    def get_velocities(self):
        wheel_diameter = self.wheel_radius * 2
        circumference = wheel_diameter * math.pi
        delta_l = self.left_tick - self.old_left
        delta_r = self.right_tick - self.old_right
        self.left_vel = delta_l / 0.1
        self.right_vel = delta_r / 0.1
    
    def pid_loop(self):
        right_err = self.target_right_ws - self.right_vel
        # ki and kd terms 0 for proportional controller
        pid_out_right = self.kp * right_err 

        left_err = self.target_left_ws - self.left_vel
        pid_out_left = self.kp * left_err

def main(args=None):
    rclpy.init(args=args)
    node = PID_node() # MODIFY NAME
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()