import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String
from sensor_msgs.msg import Image
import cv2
import pyrealsense2 as rs
import numpy as np
from cv_bridge import CvBridge
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSReliabilityPolicy

class CamPublisher(Node):

    def __init__(self):
        super().__init__('camera_pub')
        
        qos_profile = QoSProfile(
            reliability = QoSReliabilityPolicy.RELIABLE,
            history = QoSHistoryPolicy.KEEP_LAST,
            depth = 10,
            durability = QoSDurabilityPolicy.VOLATILE)  #QoS 설정
        
        self.cam_pub = self.create_publisher(Image, 'camera', qos_profile)
        self.timer = self.create_timer(0.1, self.publish_camera)
        
        self.cap = cv2.VideoCapture(0)
        self.br = CvBridge()
        
    def publish_camera(self):
        ret, frame = self.cap.read()
        w = 1920
        h = 1080
        
        # 해상도를 높이면 pub이 되다 만다..?
        # 카메라 한 번에 잡기 쉽지 않음..
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,w)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT,h)

        if ret:
            print("ret check")
            # frame = cv2.flip(frame, 0)
            self.cam_pub.publish(self.br.cv2_to_imgmsg(frame, "bgr8"))
            
        self.get_logger().info('Publishing video frame')


def main(args=None):
    rclpy.init(args=args)
    node = CamPublisher()  #클래스 실행
    try:
        rclpy.spin(node)  #ROS에 노드 spin, 프로그램 종료까지 반복
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')  #종료시 logger에 기록
    finally:
        node.destroy_node()  #노드 종료
        rclpy.shutdown()  #rclpy 종료


if __name__ == '__main__':
  main()