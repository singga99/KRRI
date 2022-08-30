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

class CamCalibration(Node):

    def __init__(self):
        super().__init__('camera_calib')
        
        qos_profile = QoSProfile(
            reliability = QoSReliabilityPolicy.RELIABLE,
            history = QoSHistoryPolicy.KEEP_LAST,
            depth = 10,
            durability = QoSDurabilityPolicy.VOLATILE)
        
        self.w = 1920
        self.h = 1080
            
        f = open("/home/krri/shin_dev/cam_calib/tmp/calibrationdata/ost.yaml")
        cal_mat = yaml.load(f, Loader = yaml.FullLoader)

        mtx = cal_mat['camera_matrix']
        cm_data = mtx['data']

        dist = cal_mat['distortion_coefficients']
        dist_data = dist['data']

        self.cameraMatrix = np.array(cm_data).reshape(3, 3)
        self.distCoeffs = np.array(dist_data).astype(np.float64)
        
        # camera
        self.cam_calib = self.create_publisher(Image, 'camera_calib', qos_profile)
        self.timer = self.create_timer(0.1, self.publish_camera)
        
        self.cap_calib = cv2.VideoCapture(0)
        self.br = CvBridge()
        
    def publish_camera(self):
        ret_calib, frame_calib = self.cap_calib.read()
        
        newcameramtx, roi = cv2.getOptimalNewCameraMatrix(self.cameraMatrix, self.distCoeffs, (self.w, self.h), 0, (self.w, self.h))
        dst = cv2.undistort(frame_calib, self.cameraMatrix, self.distCoeffs, None, newcameramtx)
        
        if ret_calib:
            self.cam_calib.publish(self.br.cv2_to_imgmsg(dst, "bgr8"))
            
        self.get_logger().info('Publishing video frame')


def main(args=None):
    rclpy.init(args=args)
    node = CamCalibration()  #클래스 실행
    try:
        rclpy.spin(node)  #ROS에 노드 spin, 프로그램 종료까지 반복
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')  #종료시 logger에 기록
    finally:
        node.destroy_node()  #노드 종료
        rclpy.shutdown()  #rclpy 종료


if __name__ == '__main__':
  main()