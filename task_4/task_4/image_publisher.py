import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')
        self.publisher_ = self.create_publisher(Image, '/video_data', 10)
        self.timer = self.create_timer(0.1, self.publish_frame)  
        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture('Object_Detection_video.avi')  #Location of the Object_detection Video

        if not self.cap.isOpened():
            self.get_logger().error("Cannot open video file 'lab3_video.avi'")
            rclpy.shutdown()

    def publish_frame(self):
        ret, frame = self.cap.read()
        if ret:
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.publisher_.publish(msg)
            self.get_logger().info('Publishing video frame')
        else:
            self.get_logger().info('End of video reached or error reading frame.')
            self.cap.release()
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    image_publisher = ImagePublisher()
    rclpy.spin(image_publisher)
    image_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
