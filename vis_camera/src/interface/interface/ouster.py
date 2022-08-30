import sys
import os
import rclpy
import time
import array
from tqdm import tqdm
import atexit # 종료처리기
import time

from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from custom_msg.msg import *
import struct
from more_itertools import nth

from ouster import client
import numpy as np
from contextlib import closing

from multiprocessing import Process, Queue


class pcl_reflection(Node):

    def __init__(self, sensorIP, mode, isStandby):
        super().__init__('pcl_reflect')
        self.publisher_reflect = self.create_publisher(PointCloud2, '/lidar/reflect', 1)
        self.publisher_imu = self.create_publisher(Lidarimu, '/lidar/imu', 1)

        self.sensorIP = sensorIP
        self.mode = mode
        self.isStandby = isStandby

        self.pcl_ref = None
        self.header_ref = None
        self.pointsBytes = None

        self.timer_callback()

    def get_sensorInfo(self):
        return self.sensorIP

    def make_xyz_parameter(self, beam_azimuth_angles, beam_altitude_angles, lidar_origin_to_beam_origin_mm):
        if self.mode in [client.LidarMode.MODE_2048x10]:
            hres = 2048#1024#
            self.maxCounter = 44*2047 #엔코더 틱 * 마지막 레이저 위치
            self.frameTick = 44*16
            self.rangeArr = np.array([0.0]*2048*64)
            self.signalArr = np.array([0]*2048*64)
        elif self.mode in [client.LidarMode.MODE_1024x20, client.LidarMode.MODE_1024x10]:
            hres = 1024
            self.maxCounter = 88*1023
            self.frameTick = 88*16
            self.rangeArr = np.array([0.0]*1024*64)
            self.signalArr = np.array([0]*1024*64)
        else:
            hres = 512
            self.maxCounter = 176*511
            self.frameTick = 176*16
            self.rangeArr = np.array([0.0]*512*64)
            self.signalArr = np.array([0]*512*64)

        vres = 64

        xFactorArr = []
        yFactorArr = []
        zFactorArr = []
        xOffsetArr = []
        yOffsetArr = []
        zOffsetArr = []
        chArr = []

        for i in tqdm(range(hres*vres)):
            hidx, vidx = divmod(i,vres)

            theta_enc = 2*np.pi*(1-hidx/hres)
            theta_azi = -2*np.pi*(beam_azimuth_angles[vidx]/360)
            pi = 2*np.pi*(beam_altitude_angles[vidx]/360)
            n = lidar_origin_to_beam_origin_mm/1000

            xFactor = np.cos(theta_enc+theta_azi)*np.cos(pi)
            yFactor = np.sin(theta_enc+theta_azi)*np.cos(pi)
            zFactor = np.sin(pi)
            xOffset = n*np.cos(theta_enc)-n*np.cos(theta_enc+theta_azi)*np.cos(pi)
            yOffset = n*np.sin(theta_enc)-n*np.sin(theta_enc+theta_azi)*np.cos(pi)
            zOffset = -n*np.sin(pi)
            #
            xFactorArr.append(xFactor)
            yFactorArr.append(yFactor)
            zFactorArr.append(zFactor)
            xOffsetArr.append(xOffset)
            yOffsetArr.append(yOffset)
            zOffsetArr.append(zOffset)
            #
            chArr.append(vidx)

        factorMatrix = np.concatenate(([xFactorArr],[yFactorArr],[zFactorArr]), axis=0)
        offsetMatrix = np.concatenate(([xOffsetArr],[yOffsetArr],[zOffsetArr]), axis=0)
        charMatrix = np.concatenate(([chArr],[chArr],[chArr]), axis=0)

        return factorMatrix, offsetMatrix, charMatrix

    def timer_callback(self):

        hostname = self.sensorIP
        config = client.SensorConfig()
        config.udp_port_lidar = 7502
        config.udp_port_imu = 7503
        config.operating_mode = client.OperatingMode.OPERATING_NORMAL
        config.lidar_mode = self.mode #client.LidarMode.MODE_2048x10

        client.set_config(hostname, config, persist=True, udp_dest_auto = True)
        print("lidar is starting please wait..")
        time.sleep(5)


        #https://static.ouster.dev/sdk-docs/api.html?highlight=imupacket#ouster.client.LidarPacket.header
        #source = timeout 1  _flush_before_read=True, latency 도 추가 https://static.ouster.dev/sdk-docs/_modules/ouster/client/core.html#Scans.stream
        with closing(client.Sensor(hostname, config.udp_port_lidar, config.udp_port_imu, buf_size=640)) as source:

            beam_azimuth_angles = (source.metadata.beam_azimuth_angles)
            beam_altitude_angles = (source.metadata.beam_altitude_angles)
            lidar_origin_to_beam_origin_mm = (source.metadata.lidar_origin_to_beam_origin_mm)

            print("cal array")
            factorMatrix, offsetMatrix, charMatrix =\
             self.make_xyz_parameter(beam_azimuth_angles, beam_altitude_angles, lidar_origin_to_beam_origin_mm)

            i= 0
            tic = time.time()
            for packet in source:  # 특정 레이어를 선택할 수 있는 기능 추가, 종료하면 라이다도 꺼지는 기능 추가
                if isinstance(packet, client.LidarPacket):

                    _range = packet.field(client.ChanField.RANGE)/1000
                    _signal = packet.field(client.ChanField.SIGNAL) #client.ChanField.REFLECTIVITY
                    _count = packet.header(client.ColHeader.ENCODER_COUNT)
                    _timer = packet.header(client.ColHeader.TIMESTAMP) #nanoseconds.
                    _frame = packet.header(client.ColHeader.FRAME_ID)
                    _status = packet.header(client.ColHeader.STATUS )

                    tickCounter = int(_count[0]/self.frameTick)

                    self.rangeArr[tickCounter*1024:(tickCounter+1)*1024] = _range.T.flatten()
                    self.signalArr[tickCounter*1024:(tickCounter+1)*1024] = _signal.T.flatten()



                    if self.maxCounter in _count:
                        # print(np.shape(self.rangeArr))
                        if i == 10:
                            toc = time.time()
                            delatTime = toc-tic
                            print("10 times :", delatTime)
                            tic=toc
                            i = 0
                        i+=1

                        # tic2 = time.time()
                        try:
                            ansArr = (self.rangeArr * factorMatrix + offsetMatrix)
                            xArr = ansArr[0,:]
                            yArr = ansArr[1,:]
                            zArr = ansArr[2,:]
                            refl_field = self.signalArr.flatten() #(64, 1024) SIGNAL == intensity!!
                            points = np.concatenate(([xArr],[yArr],[zArr],[refl_field]), axis=0).T

                            if self.pcl_ref == None:
                                print("initilize poincloud message")
                                self.pcl_ref = PointCloud2()
                                self.header_ref = Header()

                                ros_dtype = PointField.FLOAT32
                                self.dtype = np.float32
                                itemsize = np.dtype(self.dtype).itemsize
                                fields = [PointField(name=n, offset=i*itemsize, datatype=ros_dtype, count=1) for i, n in enumerate(['x','y','z','intensity'])]

                                self.header_ref.frame_id='lidar'
                                self.header_ref.stamp=self.get_clock().now().to_msg()

                                self.pcl_ref.header = self.header_ref
                                self.pcl_ref.height=1
                                self.pcl_ref.width=points.shape[0]
                                self.pcl_ref.is_dense=False
                                self.pcl_ref.is_bigendian=False
                                self.pcl_ref.fields=fields
                                self.pcl_ref.point_step=(itemsize*len(points[0]))
                                self.pcl_ref.row_step=(itemsize*len(points[0])*points.shape[0])

                            if self.pcl_ref != None:
                                sec, nano = divmod(_timer,1000000000)

                                self.pcl_ref.header.stamp.sec=sec.tolist()[0]
                                self.pcl_ref.header.stamp.nanosec=nano.tolist()[0]

                                bytes = points.astype(np.float32).tobytes()
                                int8Arr = array.array('B', bytes)
                                self.pcl_ref.data = int8Arr # 1024, 0.0671 / 2048 0.1287
                                self.publisher_reflect.publish(self.pcl_ref)

                        except ValueError as e:
                            print(e)
                            print("rangeArr : ", np.shape(self.rangeArr))
                            print("factorMatrix : ", np.shape(factorMatrix))
                            print("offsetMatrix : ", np.shape(offsetMatrix))
                            print("ROS과부하시 matrix shape매칭 안되는 오류 발생합니다.")
                            print("라이다를 받는 부분과 publish하는 부분을 나눠주세요")


                elif isinstance(packet, client.ImuPacket):
                    imu = Lidarimu()

                    imu.sys_ts = packet.sys_ts
                    imu.accel_ts = packet.accel_ts
                    imu.gyro_ts = packet.gyro_ts
                    imu.accel = array.array('d', packet.accel.tolist())#.astype(np.float32)
                    imu.angular_vel = array.array('d', packet.angular_vel.tolist())

                    self.publisher_imu.publish(imu)



@atexit.register
def destroy_lidar():
    if isStandby:
        # 절전모드에서 활성화가 안되는 경우 False로 변경우 2~3회 재시작
        print("destroy_lidar")
        hostname = sensorIP
        config = client.SensorConfig()
        config.udp_port_lidar = 7502
        config.udp_port_imu = 7503
        config.operating_mode = client.OperatingMode.OPERATING_STANDBY
        client.set_config(hostname, config, persist=True, udp_dest_auto = True)


def main():
    global sensorIP, isStandby
    # 아래 부분만 수정하세요
    sensorIP = '192.168.10.10' #'169.254.136.101'
    mode = client.LidarMode.MODE_1024x10 # #client.LidarMode.MODE_2048x10 #MODE_2048x10 MODE_1024x20 MODE_512x10 MODE_512x20
    isStandby = False # True 로 변경하면 프로그램 종료시 라이다 대기모드로
    # 프로그램 종료시 라이다를 대기모드로 변경하려면 destroy_lidar 를 수정
    # 위 부분만 수정하세요

    rclpy.init(args=None)
    pcl_pub = pcl_reflection(sensorIP, mode, isStandby)

    rclpy.spin(pcl_pub)

    pcl_pub.destroy_node()
    rclpy.shutdown()


if __name__=='__main__':
    main()
