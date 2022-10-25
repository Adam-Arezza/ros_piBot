import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point, Quaternion, PoseWithCovariance, TwistWithCovariance, Twist
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int32
from std_msgs.msg import Int32MultiArray
import math
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
import tf_transformations


class WheelOdometry(Node): 
    def __init__(self):
        super().__init__("wheel_odometry")
        self.node_name = "wheel_odometry"
        # Parameters
        self.declare_parameter("wheel_radius")
        self.declare_parameter("ticks_per_rev")
        self.declare_parameter("refresh_rate")
        self.declare_parameter("meters_per_tick")
        self.declare_parameter("wheel_base")
        self.declare_parameter("frame_id")
        self.declare_parameter("child_frame_id")
        self.ticks_per_rev = self.get_parameter("ticks_per_rev").get_parameter_value().integer_value
        self.wheel_radius = self.get_parameter("wheel_radius").get_parameter_value().double_value
        self.refresh_rate = self.get_parameter("refresh_rate").get_parameter_value().double_value
        self.meters_per_tick = self.get_parameter("meters_per_tick").get_parameter_value().double_value
        self.wheel_base = self.get_parameter("wheel_base").get_parameter_value().double_value
        self.frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        self.child_frame_id = self.get_parameter("child_frame_id").get_parameter_value().string_value

        # Publishers/Subscrtibers
        self.wheel_odometry = self.create_publisher(Odometry, "/wheel/odometry", qos_profile=10)
        self.wheel_ticks = self.create_subscription(Int32MultiArray, "/ticks", self.set_ticks, qos_profile=10)
        self.pose_timer = self.create_timer(self.refresh_rate, self.get_odom)
        
        # Variables
        self.right_ticks = 0
        self.left_ticks = 0
        self.old_right_ticks = 0
        self.old_left_ticks = 0

        # pose variables
        self.heading = 0.0
        self.x = 0.0
        self.y = 0.0

        # velocity variables
        self.x_vel = 0.0
        self.y_vel = 0.0
        self.z_angular_vel = 0.0
        
        # Initialization message
        self.get_logger().info(f'{self.node_name} is now online.')

    def set_ticks(self, ticks):
        self.right_ticks = ticks.data[0]
        self.left_ticks = ticks.data[1]

    def get_odom(self):
        # time interval between pose calculations (same as timer)
        dt = self.refresh_rate

        # the right and left changes in ticks for the interval
        d_right_ticks = self.right_ticks - self.old_right_ticks
        d_left_ticks = self.left_ticks - self.old_left_ticks

        # updating the old ticks
        self.old_left_ticks = self.left_ticks
        self.old_right_ticks = self.right_ticks

        # the distance in meters each wheel has rotated
        circumference = self.wheel_radius * 2 * math.pi
        dist_l = (d_left_ticks / self.ticks_per_rev) * circumference
        dist_r = (d_right_ticks / self.ticks_per_rev) * circumference
        dist = (dist_l + dist_r) / 2

        # the change in heading for the interval
        # d_heading = self.heading + ((dist_r - dist_l) / self.wheel_base)
        # angle_update = self.heading + ((self.right_vel - self.left_vel) / self.wheel_base)*dt
        d_heading = math.atan2(math.sin(self.heading + ((dist_r - dist_l) / self.wheel_base)), math.cos(self.heading + ((dist_r - dist_l) / self.wheel_base)))

        # the change in x and y position
        d_x = None
        d_y = None

        if d_left_ticks > 0 and d_right_ticks > 0:
            d_x = self.x + dist * math.cos(self.heading)
            d_y = self.y + dist * math.sin(self.heading)

            # updating the x, y and heading
            self.x = d_x 
            self.y = d_y 
            self.heading = d_heading 

            # updating the velocities
            self.x_vel = d_x / dt 
            self.y_vel = d_y / dt 
            self.z_angular_vel = d_heading / dt
        else:
            self.x = self.x
            self.y = self.y
            self.heading = self.heading
            self.x_vel = 0.0
            self.y_vel = 0.0
            self.z_angular_vel = 0.0

        #Constructing Odometry message and publishing odometry data
        odom_msg = Odometry()
        odom_msg.header = Header()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = self.frame_id
        odom_msg.child_frame_id = self.child_frame_id
        odom_msg.pose = PoseWithCovariance()
        odom_msg.twist = TwistWithCovariance()
        
        for i in range(36):
            if i == 0 or i == 7 or i == 14:
                odom_msg.pose.covariance[i] = 0.01
                odom_msg.twist.covariance[i] = 0.01
            elif i == 21 or i == 28 or i == 35:
                odom_msg.pose.covariance[i] += 0.01
                odom_msg.twist.covariance[i] += 0.1
            else:
                odom_msg.pose.covariance[i] = 0.0
                odom_msg.twist.covariance[i] = 0.0
        
        # Initializing msgs
        odom_msg.pose.pose = Pose()
        odom_msg.pose.pose.position = Point()
        odom_msg.pose.pose.orientation = Quaternion()

        # setting the pose variables
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = self.heading

        # setting the orientation variables
        quat = tf_transformations.quaternion_from_euler(0,0,self.heading)
        odom_msg.pose.pose.orientation.x = quat[0]
        odom_msg.pose.pose.orientation.y = quat[1]
        odom_msg.pose.pose.orientation.z = quat[2]
        odom_msg.pose.pose.orientation.w = quat[3]

        # setting the velocity variables
        odom_msg.twist.twist.linear.x = self.x_vel
        odom_msg.twist.twist.linear.y = self.y_vel
        odom_msg.twist.twist.angular.z = self.z_angular_vel

        self.wheel_odometry.publish(odom_msg)

def main(args=None):
    rclpy.init(args=args)
    node = WheelOdometry() 
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()