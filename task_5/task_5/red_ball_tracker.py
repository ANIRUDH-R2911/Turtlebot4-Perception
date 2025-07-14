import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np

class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.previous_error = 0
        self.integral = 0

    def calculate(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.previous_error) / dt
        self.previous_error = error
        return self.kp * error + self.ki * self.integral + self.kd * derivative


class RedBallTracker(Node):
    def __init__(self):
        super().__init__('red_ball_tracker')
        self.subscription = self.create_subscription(
            Image, '/camera/image_raw', self.listener_callback, 10)
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.bridge = CvBridge()

        self.center_tolerance = 50
        self.min_area = 500
        self.max_area = 15000 
        self.frame_center = None
        self.linear_speed = 0.2
        self.angular_pid = PIDController(kp=0.002, ki=0.0001, kd=0.0001)
        self.linear_pid = PIDController(kp=0.05, ki=0.001, kd=0.01) 

        self.previous_time = self.get_clock().now()
        self.ball_detected = False
        self.target_area = 10000  

    def listener_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower_red = np.array([0, 100, 100])
        upper_red = np.array([10, 255, 255])
        mask = cv2.inRange(hsv, lower_red, upper_red)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        twist = Twist()

        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(largest_contour)
            ball_center_x = x + w // 2
            ball_area = w * h

            if ball_area > self.min_area:
                self.ball_detected = True
                self.get_logger().info(f'Ball detected at: x={ball_center_x}, y={y}, w={w}, h={h}')
                current_time = self.get_clock().now()
                dt = (current_time - self.previous_time).nanoseconds / 1e9
                self.previous_time = current_time

                if self.frame_center is None:
                    self.frame_center = frame.shape[1] // 2
                error_x = self.frame_center - ball_center_x
                twist.angular.z = self.angular_pid.calculate(error_x, dt)

                if ball_area < self.target_area - 1000:
                    twist.linear.x = self.linear_pid.calculate(self.target_area - ball_area, dt)
                    self.get_logger().info("Ball is far away, moving towards it.")
                elif ball_area > self.max_area:
                    twist.linear.x = -self.linear_pid.calculate(ball_area - self.target_area, dt)
                    self.get_logger().info("Ball is too close, moving away.")
                else:
                    twist.linear.x = 0.0
                    self.get_logger().info("Ball is at the desired distance, stopping.")

        else:
            self.ball_detected = False

        if not self.ball_detected:
            twist.linear.x = 0.0  
            twist.angular.z = 0.2  
            self.get_logger().info("Searching for the ball...")

        self.publisher_.publish(twist)
        cv2.imshow('Red Ball Tracker', frame)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    red_ball_tracker = RedBallTracker()
    rclpy.spin(red_ball_tracker)
    red_ball_tracker.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

