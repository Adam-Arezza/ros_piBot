import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class DiffDriveNode(Node): 
    def __init__(self):
        super().__init__("diff_drive_controller")
        self.teleop_cmds = self.create_subscription(Twist, "/cmd_vel", self.convert_to_pwm, qos_profile=10)
        self.motion_cmds = self.create_subscription(Float32MultiArray, "/vr_vl", self.convert_to_pwm, qos_profile=10)
        self.motor_pwm_vals = self.create_publisher(Float32MultiArray, "/motor_pwm_vals", qos_profile=10)


def main(args=None):
    rclpy.init(args=args)
    node = DiffDriveNode() 
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()