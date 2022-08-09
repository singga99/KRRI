import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String
from sensor_msgs.msg import Image
import cv2
import pyrealsense2 as rs
import numpy as np
import yaml
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
            durability = QoSDurabilityPolicy.VOLATILE)
        
        ## Realsense
        # self.realsense_Dpub = self.create_publisher(Image, 'realsense_depth', qos_profile)
        # self.realsense_Cpub = self.create_publisher(Image, 'realsense_color', qos_profile)
        
        # camera
        self.cam_ori = self.create_publisher(Image, 'camera_ori', qos_profile)
        # self.cam_calib = self.create_publisher(Image, 'camera_calib', qos_profile)
        self.timer = self.create_timer(0.1, self.publish_camera)
        
        ## Realsense
        # self.pipeline = rs.pipeline()
        # config = rs.config()
        # config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        # config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        # self.pipeline.start(config)
        
        self.cap_ori = cv2.VideoCapture(0)
        # self.cap_calib = cv2.VideoCapture(0)
        self.br = CvBridge()
        
    def publish_camera(self):
        ret_ori, frame_ori = self.cap_ori.read()
        # ret_calib, frame_calib = self.cap_calib.read()
        # frames = self.pipeline.wait_for_frames()
        
        ## Realsense Depth
        # depth_frame = frames.get_depth_frame()
        # depth_image = np.asanyarray(depth_frame.get_data())
        ## Realsense Color
        # color_frame = frames.get_color_frame()
        # color_image = np.asanyarray(color_frame.get_data())
        
        # ret_real, frame_real = self.cap_realsense.read()
        
        if ret_ori:
            self.cam_ori.publish(self.br.cv2_to_imgmsg(frame_ori, "bgr8"))
            
        # if ret_calib:
        #     self.cam_calib.publish(self.br.cv2_to_imgmsg(dst, "bgr8"))

        ## Realsense Depth
        # self.realsense_Dpub.publish(self.br.cv2_to_imgmsg(depth_image, "mono16"))
        # self.realsense_Cpub.publish(self.br.cv2_to_imgmsg(color_image, "bgr8"))
            
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