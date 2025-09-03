# /root/ros2_ws/src/blue_segmentation/blue_segmentation/blue_follower_node.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np

class BlueFollowerNode(Node):
    def __init__(self):
        super().__init__('blue_follower_node')
        self.bridge = CvBridge()

        # 파라미터 선언
        self.declare_parameter('target_topic', '/blue_mask')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('linear_speed', 0.3)  # m/s
        self.declare_parameter('angular_speed', 0.5) # rad/s
        self.declare_parameter('center_threshold', 0.1) # 이미지 폭 대비 중앙 영역 비율

        # 파라미터 가져오기
        target_topic = self.get_parameter('target_topic').value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.center_threshold = self.get_parameter('center_threshold').value

        # 구독자 및 발행자 설정
        self.subscription = self.create_subscription(
            Image,
            target_topic,
            self.mask_callback,
            10)
        self.publisher_ = self.create_publisher(Twist, cmd_vel_topic, 10)

        self.get_logger().info(f"Subscribing to '{target_topic}'")
        self.get_logger().info(f"Publishing to '{cmd_vel_topic}'")

    def mask_callback(self, msg):
        """마스크 이미지를 받아 로봇 제어 명령을 생성"""
        # ROS Image 메시지를 OpenCV 이미지로 변환 (mono8)
        mask = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        h, w = mask.shape

        # 마스크 이미지에서 흰색 픽셀(객체)의 모멘트 계산
        M = cv2.moments(mask)

        cmd = Twist()

        if M['m00'] > 0:
            # 객체의 중심점(centroid) 계산
            cx = int(M['m10'] / M['m00'])
            
            # 이미지 중앙 영역 계산
            center_bound_low = w / 2 * (1 - self.center_threshold)
            center_bound_high = w / 2 * (1 + self.center_threshold)

            if cx < center_bound_low:
                # 객체가 왼쪽에 있음 -> 좌회전
                self.get_logger().info('Target on the left, turning left')
                cmd.angular.z = self.angular_speed
            elif cx > center_bound_high:
                # 객체가 오른쪽에 있음 -> 우회전
                self.get_logger().info('Target on the right, turning right')
                cmd.angular.z = -self.angular_speed
            else:
                # 객체가 중앙에 있음 -> 직진
                self.get_logger().info('Target in center, moving forward')
                cmd.linear.x = self.linear_speed
        else:
            # 감지된 객체 없음 -> 정지
            self.get_logger().info('No target detected, stopping')
            # cmd는 기본적으로 0으로 초기화되므로 별도 설정 필요 없음

        # Twist 메시지 발행
        self.publisher_.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = BlueFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
