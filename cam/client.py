#!/usr/bin/env python
# -*- coding=utf-8 -*-

import cv2
import zmq
import base64
import picamera
from picamera.array import PiRGBArray

IP = '192.168.3.18' #视频接受端的IP地址

"""初始化摄像头部分"""
camera = picamera.PiCamera()
camera.resolution = (640,480)
camera.framerate = 20
rawCapture = PiRGBArray(camera, size = (640,480))

"""实例化用来发送帧的zmq对象"""
contest = zmq.Context()
"""zmq对象使用TCP通讯协议"""
footage_socket = contest.socket(zmq.PAIR)
"""zmq对象和视频接收端建立TCP通讯协议"""
footage_socket.connect('tcp://%s:5555'%IP)
print(IP)

"""循环从摄像头采集图像
    由于使用的是树莓派摄像头，因此需要把use_video_port设置为True
    frame为采集到的图像"""
for frame in camera.capture_continuous(rawCapture, format='bgr', use_video_port=True):
    frame_image = frame.array #把采集到的图像进行转换为numpy array
    encoded, buffer = cv2.imencode('.jpg', frame_image) #把转换后的图像数据再次转换成流数据，
                                                        # 并且把流数据储存到内吨buffer中
    jpg_as_test = base64.b64encode(buffer) #把内存中的图像流数据进行base64编码
    footage_socket.send(jpg_as_test) #把编码后的流数据发送给视频的接收端
    rawCapture.truncate(0) #释放内存，准备进行下一帧的视频图像传输


