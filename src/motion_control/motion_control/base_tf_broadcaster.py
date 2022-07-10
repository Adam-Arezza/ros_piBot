from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Float32MultiArray
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
import tf_transformations
from geometry_msgs.msg import Pose2D

class FramePublisher(Node):
    def __init__(self):
        super().__init__('base_tf_broadcaster')
        self.broadcaster = TransformBroadcaster(self)
        self.pose_subscription = self.create_subscription(
            Pose2D,
            '/wheel/odometry',
            self.handle_pose,
            qos_profile=10
        )
    
    def handle_pose(self, msg):

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'base_link'

        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = 0.0

        q = msg.pose.pose.orientation
        t.transform.rotation.x = q.x
        t.transform.rotation.y = q.y
        t.transform.rotation.z = q.z
        t.transform.rotation.w = q.w
        self.broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = FramePublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
