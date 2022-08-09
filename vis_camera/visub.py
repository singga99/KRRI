import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
import numpy as np
import cv2
import open3d as o3d
from cv_bridge import CvBridge
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSReliabilityPolicy
import pointcloud2 as rnp

device = o3d.core.Device("CPU:0")
dtype = o3d.core.float32

class CamSubscriber(Node):

    def __init__(self):
        super().__init__('camera_sub')
        
        qos_profile = QoSProfile(
            reliability = QoSReliabilityPolicy.RELIABLE,
            history = QoSHistoryPolicy.KEEP_LAST,
            depth = 10,
            durability = QoSDurabilityPolicy.VOLATILE)  #QoS 설정, 큐 사이즈 10(10개의 데이터 보관)
        
        self.output_d = "/home/krri/shin_dev/vis_camera/extract/depth/"
        self.output_c = "/home/krri/shin_dev/vis_camera/extract/rgb/"
        self.d_count = 0
        self.c_count = 0
        
        # self.sub = self.create_subscription(
        #     Image,
        #     'realsense_depth',
        #     self.subscribe_depth,
        #     1)
        # self.sub = self.create_subscription(
        #     Image,
        #     'realsense_color',
        #     self.subscribe_color,
        #     1)
        
        self.sub_lidar = self.create_subscription(
            PointCloud2,
            '/lidar/reflect',
            self.subscribe_lidar,
            1)

        self.sub_cam = self.create_subscription(
            Image,
            'camera_calib',
            self.subscribe_cam_calib,
            1)
        
        self.br = CvBridge()
        
    # def subscribe_depth(self, data):
    #     depth_frame = self.br.imgmsg_to_cv2(data)
    #     cv2.imwrite("/home/krri/nas_student/신가현/vis_camera/Extract/depth/" + str(self.d_count).zfill(6) + ".png",depth_frame )
    #     self.d_count += 1
        
        
    # def subscribe_color(self, data):
    #     color_frame = self.br.imgmsg_to_cv2(data)
    #     cv2.imwrite("/home/krri/nas_student/신가현/vis_camera/Extract/color/" + str(self.c_count).zfill(6) + ".jpg",color_frame )
    #     self.c_count += 1
        
    def subscribe_lidar(self, msg):
        pcd = msg
        pcd_data = rnp.pointcloud2_to_array(pcd)
        
        xyz_name = ['x','y','z']
        msg_xyz=[]
        
        for xyz in xyz_name:
            msg_xyz.append(pcd_data[xyz].reshape(-1,1))
        
        pcd_xyz = np.concatenate(msg_xyz, axis=1)
        pcd = o3d.t.geometry.PointCloud(device)
        pcd.point['positions'] = o3d.core.Tensor(pcd_xyz, dtype, device)
        o3d.t.io.write_point_cloud(self.output_d + str(self.d_count) + ".pcd", pcd)
        self.d_count += 1    
        print(self.d_count)
        
    def subscribe_cam_calib(self, msg):
        img = self.br.imgmsg_to_cv2(msg,'bgr8')
        cv2.imwrite(self.output_c + str(self.c_count) + ".jpg", img)
        self.c_count += 1    
        
            


def main(args=None):
    rclpy.init(args=args)
    node = CamSubscriber()  #클래스 실행
    try:
        rclpy.spin(node)  #ROS에 노드 spin, 프로그램 종료까지 반복
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')  #종료시 logger에 기록
    finally:
        node.destroy_node()  #노드 종료
        rclpy.shutdown()  #rclpy 종료


if __name__ == '__main__':
  main()
