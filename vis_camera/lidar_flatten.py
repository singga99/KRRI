import ast
from sensor_msgs.msg import Image

from matplotlib.pyplot import axis
import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from sensor_msgs.msg import PointCloud2
from cv_bridge import CvBridge
import pointcloud2 as rnp
import time

class LidarFlatten(Node):

    def __init__(self):
        super().__init__('lidar_flatten')
        
        self.lidar_sub = self.create_subscription(
            PointCloud2,
            '/lidar/reflect',
            self.lidar_flatten,
            1)
        
        self.calib_sub = self.create_subscription(
            Image,
            '/camera_calib',
            self.cam_calib,
            1
        )
        
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.callback)

        self.R = np.array([ [-0.11998798,  0.98997037, -0.0745758 ],
                            [-0.02296834, -0.07786654, -0.99669918],
                            [-0.99250962, -0.11787904,  0.03208102]])
        
        self.point2D = np.array([0])
        self.img = np.array([0])
        
    def map2D(self, pc_data):
    
        x_size = 720
        y_size = 1280
        z_size = 300
        
        
        x_range = 1.5
        y_range = 2
        z_range = 10.0

        grid_size = np.array([2 * x_range / x_size, 2 * y_range / y_size, 2 * z_range / z_size])
        image_size = np.array([x_size, y_size, z_size])

        # 좌표 변환
        shifted_coord = np.asarray(pc_data)[:,[0,1,2]] # zxy
        shifted_coord[:,[0,1]] = shifted_coord[:,[0,1]]/-shifted_coord[:,[2]]
        
        shifted_coord[:,0] += x_range
        shifted_coord[:,1] += y_range
        shifted_coord[:,2] = -shifted_coord[:,2] +10
        # 위가 왜 - 가 되야하지?
        
        # image index
        index = np.floor(shifted_coord / grid_size).astype(np.int64)
        
        # choose illegal index
        bound_x = np.logical_and(index[:, 0] >= 0, index[:, 0] < image_size[0])
        bound_y = np.logical_and(index[:, 1] >= 0, index[:, 1] < image_size[1])
        bound_z = np.logical_and(index[:, 2] >= 0, index[:, 2] < image_size[2]) # 0-255
        
        bound_box = np.logical_and(np.logical_and(bound_x, bound_y), bound_z)
        
        index = index[bound_box]

        # show image  
        image = np.zeros((z_size, x_size, y_size), dtype=np.uint8)  
        image[index[:, 2], index[:, 0], index[:, 1]] = index[:, 2]
        
        image = np.amax(image, axis=0) 

        return image
    
    def map2D_1(self, pc_data):
    
        x_size = 720
        y_size = 1280
    
        x_range = 0.5
        y_range = 1


        grid_size = np.array([2 * x_range / x_size, 2 * y_range / y_size])
        image_size = np.array([x_size, y_size])

        # 좌표 변환
        shifted_coord = np.asarray(pc_data)[:,[0,1,2]] # zxy
        shifted_coord[:,[0,1]] = shifted_coord[:,[0,1]]/-shifted_coord[:,[2]]
        
        shifted_coord[:,0] += x_range
        shifted_coord[:,1] += y_range
        
        # image index
        index = np.floor(shifted_coord[:,[0,1]] / grid_size).astype(np.int64)
        
        # choose illegal index
        bound_x = np.logical_and(index[:, 0] >= 0, index[:, 0] < image_size[0])
        bound_y = np.logical_and(index[:, 1] >= 0, index[:, 1] < image_size[1])
        
        bound_box = np.logical_and(bound_x, bound_y)
        
        index = index[bound_box]

        # show image  
        image = np.zeros((x_size, y_size), dtype=np.uint8)  
        image[index[:, 0], index[:, 1]] = 255

        return image

    def rotmatrix(self, roll, pitch, yaw):
        
        yaw_rotation_matrix = np.array([
                    [np.cos(yaw), -np.sin(yaw), 0.0],
                    [np.sin(yaw), np.cos(yaw), 0.0],
                    [0.0, 0.0, 1.0]])

        pitch_rotation_matrix = np.array([
                    [1.0, 0.0, 0.0],
                    [0.0, np.cos(pitch), -np.sin(pitch)],
                    [0.0, np.sin(pitch), np.cos(pitch)]])

        roll_rotation_matrix = np.array([
                    [np.cos(roll), 0.0, np.sin(roll)],
                    [0.0, 1.0, 0.0],
                    [-np.sin(roll), 0.0, np.cos(roll)]])
        
        R = np.dot(roll_rotation_matrix, yaw_rotation_matrix)
        
        return R
        
    
    def lidar_flatten(self, msg):
        pcd = msg
        pcd_data = rnp.pointcloud2_to_array(pcd)
        
        xyz_name = ['x','y','z']
        msg_xyz=[]
        
        for xyz in xyz_name:
            msg_xyz.append(pcd_data[xyz].reshape(-1,1))
        
        pcd_xyz = np.concatenate(msg_xyz, axis=1)
            
        pcd_xyz = pcd_xyz[np.where(pcd_xyz[:,0] < 0)]
        # print(pcd_xyz[:,0].max())
        rot = self.rotmatrix(np.deg2rad(-90),0,0)
        
        rot_verts = pcd_xyz
        rot_verts = np.dot(rot, rot_verts.T).T
        
        rot_verts[:,[0]] = rot_verts[:,[0]] #+ (0.57352549)
        rot_verts[:,[1]] = rot_verts[:,[1]] #+ (-1.5)

        
        tic = time.time()
        self.point2D = self.map2D_1(rot_verts)
        toc = time.time()
        # print(toc-tic) 
        
        # cv2.imshow('lidar', self.point2D)
        
        # if cv2.waitKey(1) & 0xFF == 27:
        #     cv2.destroyAllWindows()
            
    def cam_calib(self, msg):
        br = CvBridge()
        img = br.imgmsg_to_cv2(msg,'bgr8')
        self.img = cv2.resize(img, dsize = (1280,720), interpolation = cv2.INTER_AREA)
        self.img = cv2.cvtColor(self.img,cv2.COLOR_BGR2GRAY)
        
        img2 = cv2.add(self.img, self.point2D)
        cv2.imshow("a",img2)
        
        if cv2.waitKey(1) & 0xFF == 27:
            cv2.destroyAllWindows()
            
    def callback(self):
        pass
        
        
def main(args=None):
    rclpy.init(args=args)
    node = LidarFlatten()  #클래스 실행
    try:
        rclpy.spin(node)  #ROS에 노드 spin, 프로그램 종료까지 반복
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')  #종료시 logger에 기록
    finally:
        node.destroy_node()  #노드 종료
        rclpy.shutdown()  #rclpy 종료


if __name__ == '__main__':
  main()