import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int32
from std_msgs.msg import Int32MultiArray
import math


class PoseEstimator(Node): 
    def __init__(self):
        super().__init__("pose_estimator")
        self.node_name = "pose_estimator"
        # Parameters
        self.declare_parameter("wheel_radius")
        self.declare_parameter("ticks_per_rev")
        self.declare_parameter("refresh_rate")
        self.declare_parameter("meters_per_tick")
        self.declare_parameter("wheel_base")
        self.ticks_per_rev = self.get_parameter("ticks_per_rev").get_parameter_value().integer_value
        self.wheel_radius = self.get_parameter("wheel_radius").get_parameter_value().double_value
        self.refresh_rate = self.get_parameter("refresh_rate").get_parameter_value().double_value
        self.meters_per_tick = self.get_parameter("meters_per_tick").get_parameter_value().double_value
        self.wheel_base = self.get_parameter("wheel_base").get_parameter_value().double_value

        self.robot_pose = self.create_publisher(Pose2D, "/robot_pose", qos_profile=10)
        self.imu_data = self.create_subscription(Float32MultiArray, "/imu_data", self.set_imu_vals, qos_profile=10)
        self.wheel_ticks = self.create_subscription(Int32MultiArray, "/ticks", self.set_ticks, qos_profile=10)
        self.wheel_velocities_subscriber = self.create_subscription(Float32MultiArray, "/wheel_linear_velocity", self.set_wheel_vels, qos_profile=10)
        self.pose_timer = self.create_timer(0.01, self.estimate_pose)
        
        self.right_ticks = 0
        self.left_ticks = 0
        self.old_right_ticks = 0
        self.old_left_ticks = 0
        self.x_accel = 0
        self.y_accel = 0
        self.z_accel = 0
        self.x_degrees_per_second = 0
        self.y_degrees_per_second = 0
        self.z_degrees_per_second = 0
        self.heading = 0
        self.x = 0.0
        self.y = 0.0
        self.right_vel = 0
        self.left_vel = 0
        
        # Initialization message
        self.get_logger().info(f'{self.node_name} is now online.')

    def set_imu_vals(self, imu_data):
        self.x_degrees_per_second = imu_data.data[0]
        self.y_degrees_per_second = imu_data.data[1]
        self.z_degrees_per_second = imu_data.data[2]
        self.x_accel = imu_data.data[3]
        self.y_accel = imu_data.data[4]
        self.z_accel = imu_data.data[5]

    def set_ticks(self, ticks):
        self.right_ticks = ticks.data[0]
        self.left_ticks = ticks.data[1]
    
    def set_wheel_vels(self, vels):
        self.right_vel = vels.data[0]
        self.left_vel = vels.data[1]

    def estimate_pose(self):
        # time interval between pose calculations (same as timer)
        dt = 0.01

        # the right and left changes in ticks for the interval
        d_right_ticks = self.right_ticks - self.old_right_ticks
        d_left_ticks = self.left_ticks - self.old_left_ticks

        # updating the old ticks
        self.old_left_ticks = self.left_ticks
        self.old_right_ticks = self.right_ticks

        # the distance in meters each wheel has rotated
        circumference = self.wheel_radius * 2 * math.pi
        dist_l = (d_left_ticks / 40) * circumference
        dist_r = (d_right_ticks / 40) * circumference
        dist = (dist_l + dist_r) / 2

        # the change in heading for the interval
        d_heading = self.heading + ((dist_r - dist_l) / self.wheel_base)
        # angle_update = self.heading + ((self.right_vel - self.left_vel) / self.wheel_base)*dt
        d_heading = math.atan2(math.sin(d_heading), math.cos(d_heading))
        # the change in x and y
        d_x = None
        d_y = None

        if self.right_vel > 0 and self.left_vel > 0:
            d_x = self.x + dist * math.cos(self.heading)
            d_y = self.y + dist * math.sin(self.heading)
        elif self.right_vel < 0 and self.left_vel < 0:
            d_x = self.x - dist * math.cos(self.heading)
            d_y = self.y - dist * math.sin(self.heading)
        elif self.right_vel < 0 and self.left_vel > 0:
            pass
        elif self.right_vel > 0 and self.left_vel < 0:
            pass
        elif self.right_vel == 0 and self.left_vel == 0:
            pass
        
        # updating the x, y and heading
        self.x = d_x if d_x else self.x
        self.y = d_y if d_y else self.y
        self.heading = d_heading

        # publishing the new pose
        pose = Pose2D()
        pose.x = self.x
        pose.y = self.y
        pose.theta = self.heading
        self.robot_pose.publish(pose)

def main(args=None):
    rclpy.init(args=args)
    node = PoseEstimator() 
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()