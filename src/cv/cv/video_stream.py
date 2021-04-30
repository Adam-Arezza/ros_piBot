import rclpy
from rclpy.node import Node
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class VideoStream(Node):
    def __init__(self):
        super().__init__('video_stream')
        self.node_name = 'video_stream'
        self.publish_frames = self.create_publisher(Image, 'video_frames', qos_profile=10)
        self.rate = 0.1
        self.timer = self.create_timer(self.rate, self.get_frames)
        self.capture = cv2.VideoCapture(0)
        self.bridge = CvBridge()
        self.get_logger().info(f'{self.node_name} is now online.')
    
    def get_frames(self):
        ret, frame = self.capture.read()
        if ret == True:
            self.publish_frames.publish(self.bridge.cv2_to_imgmsg(frame))       
        self.get_logger().info('Publishing video frames')

def main(args=None):
    rclpy.init(args=args)
    video = VideoStream()
    rclpy.spin(video)
    video.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()