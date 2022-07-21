import pyrealsense2 as rs
import numpy as np
import cv2
import open3d as o3d
import time

from sympy import HadamardPower

pipeline = rs.pipeline()
config = rs.config()

pipeline_wrapper = rs.pipeline_wrapper(pipeline)
pipeline_profile = config.resolve(pipeline_wrapper)
device = pipeline_profile.get_device()

config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

pc = rs.pointcloud()
pipeline.start(config)

tripod_height = 1.35

def map2D(pc_data):
    
    x_size = 50
    y_size = 50
    z_size = 255
    
    x_range = 1.0 #x_size*0.04 #20.0
    y_range = 1.0 #_size*0.04 #60.0
    z_range = 1.0

    grid_size = np.array([2 * x_range / x_size, 2 * y_range / y_size, 2 * z_range / z_size])
    image_size = np.array([x_size, y_size, z_size])

    # 좌표 변환
    shifted_coord = np.asarray(pc_data)[:,[2,0,1]] # zxy
    # shifted_coord[:,0] += x_range
    shifted_coord[:,1] += y_range
    shifted_coord[:,2] = -shifted_coord[:,2] + tripod_height

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
    print(image)

    return image

def rotmatrix(theta):
    c,s = np.cos(theta), np.sin(theta)
    R = np.array(((c, -s), (s, c)))
    return R

rot = rotmatrix(np.deg2rad(-45))
prevTime = 0

try:
    while True:
        
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        
        if not depth_frame:
            continue

        points = pc.calculate(depth_frame)
        # print(points)
        
        v = points.get_vertices()
        verts = np.asanyarray(v).view(np.float32).reshape(-1, 3)  # xyz
        
        rot_verts = verts
        rot_verts[:,1:3] = np.dot(rot, rot_verts[:, 1:3].T).T # yz 0.007
        
 
        # print(verts)
        data = map2D(rot_verts) # 0.24
        
        data = cv2.resize(data, (500, 500))
        # print(data)
        cv2.imshow('RealSense', data)
        
        key = cv2.waitKey(1)
        if key == 27:
            cv2.destroyAllWindows()
            break
        
finally:
    pipeline.stop()
