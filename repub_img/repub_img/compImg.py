import sys
import os
import rclpy
from rclpy.node import Node

import cv2
from cv_bridge import CvBridge

from custom_msg.msg import *
from sensor_msgs.msg import CompressedImage, Image

import time


class compImgPublisher(Node):

    def __init__(self):
        super().__init__('comp_img_publisher')  
        
        self.subscription_1 = self.create_subscription(
            Image,
            '/usb_cam_1/image_raw',
            self.listener_callback_1,
            1)
        self.subscription_1  # prevent unused variable warning
        
        self.subscription_2 = self.create_subscription(
            Image,
            '/usb_cam_2/image_raw',
            self.listener_callback_2,
            1)
        self.subscription_2  # prevent unused variable warning
        
        self.publisher_1 = self.create_publisher(CompressedImage, '/usb_cam_1/image_comp', 1)
        self.publisher_2 = self.create_publisher(CompressedImage, '/usb_cam_2/image_comp', 1)
        
        self.br = CvBridge()
        
        
    def listener_callback_1(self, msg):
        
        img = self.br.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        #img = cv2.resize(img, dsize=(1280, 720), interpolation=cv2.INTER_AREA)
        cmprsmsg = self.br.cv2_to_compressed_imgmsg(img, dst_format='jpeg')
        
        cmprsmsg.header = msg.header
        
        self.publisher_1.publish(cmprsmsg)
        
       # encode_param=[int(cv2.IMWRITE_JPEG_QUALITY),80]
       # result,encimg=cv2.imencode('.jpg',img,encode_param)
        
    def listener_callback_2(self, msg):
        
        img = self.br.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        #img = cv2.resize(img, dsize=(1280, 720), interpolation=cv2.INTER_AREA)
        cmprsmsg = self.br.cv2_to_compressed_imgmsg(img, dst_format='jpeg')
        
        cmprsmsg.header = msg.header
        
        self.publisher_2.publish(cmprsmsg)



def main(args=None):
    rclpy.init(args=args)

    comp_img_publisher = compImgPublisher()
    rclpy.spin(comp_img_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()
    ch.busOff()


if __name__ == '__main__':
    main()
