import rclpy
import math
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int32


class PID_node(Node):
    def __init__(self):
        super().__init__("pid_controller")
        self.declare_parameter("kp")
        self.declare_parameter("ki")
        self.declare_parameter("kd")
        self.declare_parameter("pid_rate")
        self.kp = self.get_parameter("kp").get_parameter_value().double_value
        self.ki = self.get_parameter("ki").get_parameter_value().double_value
        self.kd = self.get_parameter("kd").get_parameter_value().double_value
        self.pid_rate = self.get_parameter("pid_rate").get_parameter_value().double_value
        self.pid_timer = self.create_timer(self.pid_rate, self.pid_loop)
        self.target_vels = self.create_subscription(Float32MultiArray, "/vr_vl", self.set_targets)
        self.current_wheel_vels = self.create_subscription(Float32MultiArray, "/velR_velL", self.set_vels)
        self.pid_output = self.create_publisher(Float32MultiArray, "/pidR_pidL", qos_profile=10)
        self.target_right_vel = 0
        self.target_left_vel = 0
        self.right_vel = 0
        self.left_vel = 0
        self.get_logger().info("PID controller has started running...")

    def set_targets(self, targets):
        self.target_right_vel = targets.data[0]
        self.target_left_vel = targets.data[1]
    
    def set_vels(self, vels):
        self.right_vel = vels.data[0] 
        self.left_vel = vels.data[1] 

    def pid_loop(self):
        right_err = self.target_right_vel - self.right_vel
        # ki and kd terms 0 for proportional controller
        pid_out_right = self.kp * right_err
        left_err = self.target_left_vel - self.left_vel
        pid_out_left = self.kp * left_err
        pid_out = Float32MultiArray()
        pid_out.data = [pid_out_right, pid_out_left]
        self.pid_output.publish(pid_out)


def main(args=None):
    rclpy.init(args=args)
    node = PID_node()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
