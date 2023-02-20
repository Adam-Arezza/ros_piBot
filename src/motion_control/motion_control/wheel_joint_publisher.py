import rclpy
import math
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Header
from std_msgs.msg import Int32MultiArray

class WheelJointNode(Node):
    def __init__(self):
        super().__init__("wheel_joints")
        self.node_name = "wheel_joints"
        self.left_ticks = 0
        self.right_ticks = 0
        self.ticks_per_rev = 40
        self.rad_per_tick = 0.157079
        self.right_rads = 0
        self.left_rads = 0
        self.wheel_ticks = self.create_subscription(Int32MultiArray, "/ticks", self.set_ticks, qos_profile=10)
        self.wheel_joint_pub = self.create_publisher(JointState, "/wheel_states", qos_profile=10)
        self.get_logger().info(f"Wheel joint state publisher online.")

    def set_ticks(self, ticks):
        new_right_ticks = ticks.data[0] - self.right_ticks
        new_left_ticks = ticks.data[1] - self.left_ticks

        self.right_rads += new_right_ticks * self.rad_per_tick
        self.left_rads += new_left_ticks * self.rad_per_tick
        if self.right_rads > self.ticks_per_rev * self.rad_per_tick:
            self.right_rads = self.right_rads - (self.ticks_per_rev * self.rad_per_tick)

        if self.left_rads > self.ticks_per_rev * self.rad_per_tick:
            self.left_rads = self.left_rads - (self.ticks_per_rev * self.rad_per_tick)

        wheel_joints_msg = JointState()
        wheel_joints_msg.header = Header()
        wheel_joints_msg.header.stamp = self.get_clock().now().to_msg()
        wheel_joints_msg.header.frame_id = "base_link"
        wheel_joints_msg.name = ["drivewhl_r_joint", "drivewhl_l_joint"]
        wheel_joints_msg.position = [self.right_rads, self.left_rads]
        wheel_joints_msg.velocity = []
        wheel_joints_msg.effort = []
        self.wheel_joint_pub.publish(wheel_joints_msg)


        self.right_ticks = ticks.data[0]
        self.left_ticks = ticks.data[1]

def main(args=None):
    rclpy.init(args=args)
    node = WheelJointNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
