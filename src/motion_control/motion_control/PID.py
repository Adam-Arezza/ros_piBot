import rclpy
import math
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int32


class PID_node(Node):
    def __init__(self):
        super().__init__("pid_controller")
        self.node_name = "pid_controller"

        # Parameters
        self.declare_parameter("kp")
        self.declare_parameter("ki")
        self.declare_parameter("kd")
        self.declare_parameter("pid_rate")
        self.kp = self.get_parameter("kp").get_parameter_value().double_value
        self.ki = self.get_parameter("ki").get_parameter_value().double_value
        self.kd = self.get_parameter("kd").get_parameter_value().double_value
        self.pid_rate = self.get_parameter("pid_rate").get_parameter_value().double_value

        # Timers
        self.pid_timer = self.create_timer(self.pid_rate, self.pid_loop)

        # Subcribers and Publishers
        self.target_vels = self.create_subscription(Float32MultiArray, "/vr_vl", self.set_targets, qos_profile=10)
        self.current_wheel_vels = self.create_subscription(Float32MultiArray, "/velocity", self.set_vels, qos_profile=10)
        self.pid_output = self.create_publisher(Float32MultiArray, "/pidR_pidL", qos_profile=10)

        # Vairables
        self.target_right_vel = 0
        self.target_left_vel = 0
        self.right_vel = 0
        self.left_vel = 0
        self.right_err_sum = 0
        self.left_err_sum = 0

        # Initialization message
        self.get_logger().info(f'{self.node_name} is now online.')

    def set_targets(self, targets):
        self.target_right_vel = targets.data[0]
        self.target_left_vel = targets.data[1]
    
    def set_vels(self, vels):
        self.right_vel = vels.data[0] 
        self.left_vel = vels.data[1] 

    def pid_loop(self):
        right_err = self.target_right_vel - self.right_vel
        self.right_err_sum = self.right_err_sum + right_err
        # ki and kd terms 0 for proportional controller
        pid_out_right = self.kp * right_err + self.ki
        left_err = self.target_left_vel - self.left_vel
        self.left_err_sum = self.left_err_sum + left_err
        pid_out_left = self.kp * left_err
        pid_out = Float32MultiArray()
        pid_out.data = [pid_out_right, pid_out_left]
        if self.target_right_vel == 0 and self.target_left_vel == 0:
            pid_out.data = [0.0,0.0]
        self.pid_output.publish(pid_out)


def main(args=None):
    rclpy.init(args=args)
    node = PID_node()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
