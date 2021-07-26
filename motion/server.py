# -*- coding: utf-8 -*-
"""
Created on Sat Jul 10 21:52:21 2021

@author: qq735
"""

from multiprocessing import Process
from socket import *
# import wiringpi 

import RPi.GPIO as GPIO
import time
import string
import threading
import timeout_decorator

# 按键值定义
run_car = '1'  # 按键前
back_car = '2'  # 按键后
left_car = '3'  # 按键左
right_car = '4'  # 按键右
stop_car = '0'  # 按键停

# 舵机按键值定义
front_left_servo = '1'  # 前舵机向左
front_right_servo = '2'  # 前舵机向右
up_servo = '3'  # 摄像头舵机向上
down_servo = '4'  # 摄像头舵机向下
left_servo = '6'  # 摄像头舵机向左
right_servo = '7'  # 摄像头舵机向右
updowninit_servo = '5'  # 摄像头舵机上下复位
stop_servo = '8'  # 舵机停止

# 小车状态值定义
enSTOP = 0
enRUN = 1
enBACK = 2
enLEFT = 3
enRIGHT = 4
enTLEFT = 5
enTRIGHT = 6

# 小车舵机定义
enFRONTSERVOLEFT = 1
enFRONTSERVORIGHT = 2
enSERVOUP = 3
enSERVODOWN = 4
enSERVOUPDOWNINIT = 5
enSERVOLEFT = 6
enSERVORIGHT = 7
enSERVOSTOP = 8

# 初始化上下左右角度为90度
ServoLeftRightPos = 90
ServoUpDownPos = 90
g_frontServoPos = 90
g_nowfrontPos = 0

# 小车电机引脚定义
IN1 = 20
IN2 = 21
IN3 = 19
IN4 = 26
ENA = 16
ENB = 13

# 小车按键定义
key = 8

# 超声波引脚定义
EchoPin = 0
TrigPin = 1

# RGB三色灯引脚定义
LED_R = 22
LED_G = 27
LED_B = 24

# 舵机引脚定义
FrontServoPin = 23
ServoUpDownPin = 9
ServoLeftRightPin = 11

# 红外避障引脚定义
AvoidSensorLeft = 12
AvoidSensorRight = 17

# 蜂鸣器引脚定义
buzzer = 8

# 灭火电机引脚设置
OutfirePin = 2

# 循迹红外引脚定义
# TrackSensorLeftPin1 TrackSensorLeftPin2 TrackSensorRightPin1 TrackSensorRightPin2
#      3                 5                  4                   18
TrackSensorLeftPin1 = 3  # 定义左边第一个循迹红外传感器引脚为3口
TrackSensorLeftPin2 = 5  # 定义左边第二个循迹红外传感器引脚为5口
TrackSensorRightPin1 = 4  # 定义右边第一个循迹红外传感器引脚为4口
TrackSensorRightPin2 = 18  # 定义右边第二个循迹红外传感器引脚为18口

# 光敏电阻引脚定义
LdrSensorLeft = 7
LdrSensorRight = 6

# 变量的定义
# 七彩灯RGB三色变量定义
red = 0
green = 0
blue = 0
# TCP通信数据包标志位以及接受和发送数据变量
NewLineReceived = 0
InputString = ''
recvbuf = ''
ReturnTemp = ''
# 小车和舵机状态变量
g_CarState = 0
g_ServoState = 0
# 小车速度变量
CarSpeedControl = 1000
# 寻迹，避障，寻光变量
infrared_track_value = ''
infrared_avoid_value = ''
LDR_value = ''
g_lednum = 0

# 设置GPIO口为BCM编码方式
GPIO.setmode(GPIO.BCM)

# 忽略警告信息
GPIO.setwarnings(False)

import requests
import eventlet
import time

eventlet.monkey_patch()




# 电机引脚初始化操作
def init():
    global pwm_ENA
    global pwm_ENB
    GPIO.setup(ENA, GPIO.OUT, initial=GPIO.HIGH)
    GPIO.setup(IN1, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(IN2, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(ENB, GPIO.OUT, initial=GPIO.HIGH)
    GPIO.setup(IN3, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(IN4, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(buzzer, GPIO.OUT, initial=GPIO.HIGH)
    # 设置pwm引脚和频率为2000hz
    pwm_ENA = GPIO.PWM(ENA, 1000)
    pwm_ENB = GPIO.PWM(ENB, 1000)


# 小车前进
@timeout_decorator.timeout(1)
def run():
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    # 启动PWM设置占空比为100（0--100）
    pwm_ENA.start(100)
    pwm_ENB.start(100)

# 小车后退
def back():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)
    pwm_ENA.ChangeDutyCycle(CarSpeedControl)
    pwm_ENB.ChangeDutyCycle(CarSpeedControl)
    time.sleep(1)
    brake()


# 小车左转
def left():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    pwm_ENA.ChangeDutyCycle(CarSpeedControl)
    pwm_ENB.ChangeDutyCycle(CarSpeedControl)


# 小车右转
def right():
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.LOW)
    pwm_ENA.ChangeDutyCycle(CarSpeedControl)
    pwm_ENB.ChangeDutyCycle(CarSpeedControl)


# 小车原地左转
def spin_left():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    pwm_ENA.ChangeDutyCycle(CarSpeedControl)
    pwm_ENB.ChangeDutyCycle(CarSpeedControl)


# 小车原地右转
def spin_right():
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)
    pwm_ENA.ChangeDutyCycle(CarSpeedControl)
    pwm_ENB.ChangeDutyCycle(CarSpeedControl)


# 小车停止
def brake():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.LOW)

#
def whistle():
    GPIO.output(buzzer, GPIO.LOW)
    time.sleep(0.1)
    GPIO.output(buzzer, GPIO.HIGH)
    time.sleep(0.001)

################################################################ 需要为客户端提供服务
def do_service(connect_socket):
    while True:
        recv_data = connect_socket.recv(1024)
        if len(recv_data) == 0:
            # 发送方关闭tcp的连接,recv()不会阻塞，而是直接返回''
            # print('client %s close' % str(client_addr))     
            # s.getpeername()   s.getsockname()
            # wiringpi.digitalWrite(0,0)
            print('client %s close' % str(connect_socket.getpeername()))
            break

        if (len(recv_data) == 1) and (recv_data.decode('gbk')[0] == 'w'):
            with eventlet.Timeout(1, False):
                run()
        elif (len(recv_data) == 1) and (recv_data.decode('gbk')[0] == 's'):
            with eventlet.Timeout(1, False):
                back()
        #     continue
        # elif (len(recv_data) == 1) and (recv_data.decode('gbk')[0] == 'a'):
        #     left()
        # elif (len(recv_data) == 1) and (recv_data.decode('gbk')[0] == 'd'):
        #     right()
        # elif (len(recv_data) == 1) and (recv_data.decode('gbk')[0] == 'x'):
        #     brake()
        # # else:
        # wiringpi.digitalWrite(0,0)
        # if len(recv_data) > 1:
        # wiringpi.digitalWrite(0,0)


        print('recv: %s' % recv_data.decode('gbk'))


def main():
    init()
    # 0.init wiringpi
    # wiringpi.wiringPiSetup()
    # wiringpi.pinMode(0,1)
    # 1.创建socket
    listen_socket = socket(AF_INET, SOCK_STREAM)
    # stream流式套接字,对应tcp

    # 设置允许复用地址,当建立连接之后服务器先关闭，设置地址复用
    # 设置socket层属性    复用地址，不用等2msl，    允许
    listen_socket.setsockopt(SOL_SOCKET, SO_REUSEADDR, 1)

    # 2.绑定端口
    my_addr = ('192.168.3.24', 8888)
    listen_socket.bind(my_addr)

    # 3，接听状态
    listen_socket.listen(4)  # 设置套接字成监听,4表示一个己连接队列长度
    print('listening...')

    # 4.等待客户端来请求

    # 父进程只专注接受连接请求
    while True:
        # 接受连接请求，创建连接套接字，用于客户端间通信
        connect_socket, client_addr = listen_socket.accept()  # accept默认会引起阻塞
        # 新创建连接用的socket, 客户端的地址
        # print(connect_socket)
        print(client_addr)

        # 每当来新的客户端连接，创建子进程，由子进程和客户端通信
        process_do_service = Process(target=do_service, args=(connect_socket,))
        process_do_service.start()

        # 父进程，关闭connect_socket
        connect_socket.close()



if __name__ == "__main__":
    main()
