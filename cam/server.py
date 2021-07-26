#!/usr/bin/env python
# -*- coding=utf-8 -*-

import cv2
import zmq
import base64
import numpy as np

"""实例化用来接收帧的zmq对象"""
context = zmq.Context()
"""zmq对象建立TCP链接"""
footage_socket = context.socket(zmq.PAIR)
footage_socket.bind('tcp://*:5555')
if __name__ == '__main__':
    
    while True:
        print("listion")
        frame = footage_socket.recv_string() #接收TCP传输过来的一帧视频图像数据
        img = base64.b64decode(frame) #把数据进行base64解码后储存到内存img变量中
        npimg = np.frombuffer(img, dtype=np.uint8) #把这段缓存解码成一维数组
        source = cv2.imdecode(npimg, 1) #将一维数组解码为图像source
        cv2.imshow("Stream", source) #把图像显示在窗口中
        cv2.waitKey(1) #延时等待，防止出现窗口无响应
