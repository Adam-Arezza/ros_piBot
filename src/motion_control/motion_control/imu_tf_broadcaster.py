from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Float32MultiArray
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
import tf_transformations
from geometry_msgs.msg import Pose2D

class ImuFramePublisher(Node):
    def __init__(self):
        super().__init__('imu_tf_broadcaster')
        self.broadcaster = TransformBroadcaster(self)
        self.imu_subscriber = self.create_subscription(
            Float32MultiArray,
            '/imu/data_raw',
            self.get_rates,
            qos_profile=10
        )
        self.broadcast_timer = self.create_timer(0.02, self.handle_imu)
        self.x_rot_rate = 0.0
        self.y_rot_rate = 0.0
        self.z_rot_rate = 0.0

    def get_rates(self,imu_data):
        self.x_rot_rate = imu_data.data[0]
        self.y_rot_rate = imu_data.data[1]
        self.z_rot_rate = imu_data.data[2]

    def handle_imu(self):
        x,y,z = 0.0,0.0,0.0
        if(abs(self.x_rot_rate) > 0.01):
            x = self.x_rot_rate * 0.02
        if(abs(self.y_rot_rate) > 0.01):
            y = self.y_rot_rate * 0.02
        if(abs(self.z_rot_rate) > 0.01):
            z = self.z_rot_rate * 0.02
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'imu_link'

        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0

        q = tf_transformations.quaternion_from_euler(y,x,z)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = ImuFramePublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
