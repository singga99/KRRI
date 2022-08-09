from pyrsistent import field
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message
import rosbag2_py
import os
import numpy as np
import cv2
import pandas as pd
from cv_bridge import CvBridge


class ReadBag():
    def __init__(self):
        self.d_count = 329
        self.c_count = 329
        self.topic_list = ["/realsense_depth","/realsense_color"]
        self.br = CvBridge()

    # read bag
    def run(self):
        options = rosbag2_py.StorageOptions('','')
        options.uri = "/home/krri/nas_student/신가현/vis_camera/bag/rosbag2_2022_07_08-13_04_56"
        options.storage_id = 'sqlite3'

        converter = rosbag2_py.ConverterOptions('','')
        converter.input_serialization_format = 'cdr'
        converter.output_serialization_format = 'cdr'

        reader = rosbag2_py.SequentialReader()
        
        try:
            reader.open(options, converter)
        except Exception as e:
            print("error")

        topic_types = reader.get_all_topics_and_types()
        type_map = {topic_types[i].name : topic_types[i].type for i in range(len(topic_types))}

        storage_filter = rosbag2_py.StorageFilter(self.topic_list)
        reader.set_filter(storage_filter)

        while reader.has_next():
            (topic, data, timestamp) = reader.read_next()
            msg_type = get_message(type_map[topic])
            msg = deserialize_message(data, msg_type)
            
            if(topic in self.topic_list[0]):
                depth_frame = self.br.imgmsg_to_cv2(msg)
                cv2.imwrite("/home/krri/nas_student/신가현/vis_camera/Extract/depth/" + str(self.d_count).zfill(6) + ".png",depth_frame )
                self.d_count +=1
                
            if(topic in self.topic_list[1]):
                color_frame = self.br.imgmsg_to_cv2(msg)
                cv2.imwrite("/home/krri/nas_student/신가현/vis_camera/Extract/color/" + str(self.c_count).zfill(6) + ".jpg",color_frame )
                self.c_count +=1

            print(f"{self.d_count} {self.c_count}", end="\r")
            

# main func make dir before read bag
def main():
    readbag = ReadBag()
    readbag.run()
