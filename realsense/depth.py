from re import A
import pyrealsense2 as rs
import numpy as np
import cv2

pipeline = rs.pipeline()
config = rs.config()

config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

pipeline.start(config)
max_depth = 7

try:
    while True:
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        
        depth_image = np.asanyarray(depth_frame.get_data())
 
        # data = cv2.resize(depth_image, (640, 480))
        coverage = []
        for y in range(480):
            for x in range(640):
                dist = depth_frame.get_distance(x, y)
                coverage.append(dist)
                
        coverage = np.array(coverage).reshape(480,640)
        print(depth_image)
                
        cv2.imshow('RealSense', coverage)

        
        if cv2.waitKey(1) & 0xFF == 27: # esc 키를 누르면 닫음
            break

finally:
    pipeline.stop()