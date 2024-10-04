import rclpy
from rclpy.node import Node
import cv2
import base64
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import String
#import nanocamera as nano

class VideoStream(Node):
    def __init__(self):
        super().__init__('video_stream')
        self.node_name = 'video_stream'

        #self.publish_frames = self.create_publisher(Image, '/video_frames', qos_profile=10)
        self.publish_frames = self.create_publisher(String, '/video_frames', qos_profile=10)

        self.rate = 0.01
        self.timer = self.create_timer(self.rate, self.get_frames)
        self.capture = cv2.VideoCapture("/dev/video0")
        self.capture.set(cv2.CAP_PROP_FPS, 20)
        self.bridge = CvBridge()
        self.get_logger().info(f'{self.node_name} is now online.')
        if not self.capture:
            self.get_logger().info("Could not open camera.")
    
    def get_frames(self):
        ret, frame = self.capture.read()
        if ret:
            mod_frame = cv2.resize(frame, (320, 240))
            _, buf = cv2.imencode('.jpg', mod_frame)
            img_as_string = base64.b64encode(buf).decode('utf-8')
            pub_string = String()
            pub_string.data = img_as_string
            self.publish_frames.publish(pub_string)
            #self.publish_frames.publish(self.bridge.cv2_to_imgmsg(mod_frame, encoding='bgr8'))       
        else:
            self.get_logger().info("no frames")

def main(args=None):
    rclpy.init(args=args)
    video = VideoStream()
    rclpy.spin(video)
    video.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
