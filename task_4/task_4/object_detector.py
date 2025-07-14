import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import BoundingBox2D
from cv_bridge import CvBridge
import cv2
import numpy as np


class ObjectDetector(Node):
    def __init__(self):
        super().__init__('object_detector')
        self.subscription = self.create_subscription(
            Image, '/video_data', self.image_callback, 10)
        self.publisher_ = self.create_publisher(BoundingBox2D, '/bbox', 10)
        self.bridge = CvBridge()

    def image_callback(self, msg):
        
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(gray, 100, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)

        
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
        
            x, y, w, h = cv2.boundingRect(contour)

            
            cx = int(x + w / 2)
            cy = int(y + h / 2)

            
            self.get_logger().info(f'Centroid at: ({cx}, {cy}), Width: {w}, Height: {h}')

            
            bbox_msg = BoundingBox2D()
            bbox_msg.center.position.x = float(cx)
            bbox_msg.center.position.y = float(cy)
            bbox_msg.size_x = float(w)
            bbox_msg.size_y = float(h)

            self.publisher_.publish(bbox_msg)

            
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

       
        cv2.imshow('Object Detector', frame)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    object_detector = ObjectDetector()
    rclpy.spin(object_detector)
    object_detector.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
