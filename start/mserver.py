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

import serial
import pynmea2

serialPort = serial.Serial("/dev/ttyUSB0", 9600, timeout=0.5)

from ctypes import *
import numpy as np

mlx90640 = cdll.LoadLibrary('./libmlx90640.so')

# 初始化上下左右角度为90度
ServoLeftRightPos = 90
ServoUpDownPos = 90
g_frontServoPos = 90
nowfrontPos = 0

# 小车电机引脚定义
IN1 = 20
IN2 = 21
IN3 = 19
IN4 = 26
ENA = 16
ENB = 13

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
# 小车速度变量,20表示40cm每秒
CarSpeedControl = 20
# 寻迹，避障，寻光变量
infrared_track_value = ''
infrared_avoid_value = ''
LDR_value = ''
g_lednum = 0

# 设置GPIO口为BCM编码方式
GPIO.setmode(GPIO.BCM)

# 忽略警告信息
GPIO.setwarnings(False)

import eventlet
import time

eventlet.monkey_patch()


# 电机引脚初始化操作
def init():
    global pwm_ENA
    global pwm_ENB
    global delaytime
    global CarSpeedControl
    global pwm_FrontServo
    global pwm_UpDownServo
    global pwm_LeftRightServo
    global nowfrontPos

    GPIO.setup(ENA, GPIO.OUT, initial=GPIO.HIGH)
    GPIO.setup(IN1, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(IN2, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(ENB, GPIO.OUT, initial=GPIO.HIGH)
    GPIO.setup(IN3, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(IN4, GPIO.OUT, initial=GPIO.LOW)

    GPIO.setup(buzzer, GPIO.OUT, initial=GPIO.HIGH)

    GPIO.setup(EchoPin, GPIO.IN)
    GPIO.setup(TrigPin, GPIO.OUT)
    GPIO.setup(FrontServoPin, GPIO.OUT)
    GPIO.setup(ServoUpDownPin, GPIO.OUT)
    GPIO.setup(ServoLeftRightPin, GPIO.OUT)

    GPIO.setup(LED_R, GPIO.OUT)
    GPIO.setup(LED_G, GPIO.OUT)
    GPIO.setup(LED_B, GPIO.OUT)

    # 设置pwm引脚和频率为2000hz
    pwm_ENA = GPIO.PWM(ENA, 2000)
    pwm_ENB = GPIO.PWM(ENB, 2000)
    # pwm_ENA.start(0)
    # pwm_ENB.start(0)

    pwm_FrontServo = GPIO.PWM(FrontServoPin, 50)
    pwm_UpDownServo = GPIO.PWM(ServoUpDownPin, 50)
    pwm_LeftRightServo = GPIO.PWM(ServoLeftRightPin, 50)
    pwm_FrontServo.start(0)
    pwm_UpDownServo.start(0)
    pwm_LeftRightServo.start(0)


# 红外
def tcam():
    temp = (c_float * 768)()
    ptemp = pointer(temp)
    mlx90640.get_mlx90640_temp(ptemp)
    my_nparray = np.frombuffer(temp, dtype=np.float32)

    t = my_nparray.reshape((32, 24))

    # print(np.max(t))
    # print(np.argmax(t))
    return np.max(t), np.argmax(t) % 32


# GPS 经纬高
def GetGPS():
    lat = -1
    lon = -1
    alt = -1
    s = serialPort.readline()
    # print(s)
    # print(type(s.decode()))
    # print(s.find(b'GGA'))
    s = s.decode()
    if s.find('GGA') > -1:
        msg = pynmea2.parse(s)
        lat = msg.lat
        lon = msg.lon
        alt = msg.altitude
    return (lat, lon, alt)


# 超声波测距函数
def Distance_test():
    GPIO.output(TrigPin, GPIO.HIGH)
    time.sleep(0.000015)
    GPIO.output(TrigPin, GPIO.LOW)
    while not GPIO.input(EchoPin):
        pass
        t1 = time.time()
    while GPIO.input(EchoPin):
        pass
        t2 = time.time()
    # print ("distance is %d " % (((t2 - t1)* 340 / 2) * 100))
    time.sleep(0.01)
    return ((t2 - t1) * 340 / 2) * 100


# 根据转动的角度来点亮相应的颜色
def corlor_light(pos):
    if pos > 150:
        GPIO.output(LED_R, GPIO.HIGH)
        GPIO.output(LED_G, GPIO.LOW)
        GPIO.output(LED_B, GPIO.LOW)
    elif pos > 125:
        GPIO.output(LED_R, GPIO.LOW)
        GPIO.output(LED_G, GPIO.HIGH)
        GPIO.output(LED_B, GPIO.LOW)
    elif pos > 100:
        GPIO.output(LED_R, GPIO.LOW)
        GPIO.output(LED_G, GPIO.LOW)
        GPIO.output(LED_B, GPIO.HIGH)
    elif pos > 75:
        GPIO.output(LED_R, GPIO.HIGH)
        GPIO.output(LED_G, GPIO.HIGH)
        GPIO.output(LED_B, GPIO.LOW)
    elif pos > 50:
        GPIO.output(LED_R, GPIO.LOW)
        GPIO.output(LED_G, GPIO.HIGH)
        GPIO.output(LED_B, GPIO.HIGH)
    elif pos > 25:
        GPIO.output(LED_R, GPIO.HIGH)
        GPIO.output(LED_G, GPIO.LOW)
        GPIO.output(LED_B, GPIO.HIGH)
    elif pos > 0:
        GPIO.output(LED_R, GPIO.HIGH)
        GPIO.output(LED_G, GPIO.HIGH)
        GPIO.output(LED_B, GPIO.HIGH)
    else:
        GPIO.output(LED_R, GPIO.LOW)
        GPIO.output(LED_G, GPIO.LOW)
        GPIO.output(LED_B, GPIO.LOW)


# 舵机来回转动
def servo_control_color():
    for pos in range(19):
        frontservo_appointed_detection(pos * 10)
        time.sleep(0.02)
        updownservo_appointed_detection(pos * 10)
        time.sleep(0.02)
        leftrightservo_appointed_detection(pos * 10)
        time.sleep(0.02)
        corlor_light(pos)
    for pos in reversed(range(19)):
        frontservo_appointed_detection(pos * 10)
        time.sleep(0.02)
        updownservo_appointed_detection(pos * 10)
        time.sleep(0.02)
        leftrightservo_appointed_detection(pos * 10)
        time.sleep(0.02)
        corlor_light(pos)


# 小车前进
def run():
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    # 启动PWM设置占空比为100（0--100）
    pwm_ENA.start(CarSpeedControl)
    pwm_ENB.start(CarSpeedControl)
    print('runrun')
    # pwm_ENA.ChangeDutyCycle(CarSpeedControl)
    # pwm_ENB.ChangeDutyCycle(CarSpeedControl)


# 小车后退
def back():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)
    pwm_ENA.ChangeDutyCycle(CarSpeedControl)
    pwm_ENB.ChangeDutyCycle(CarSpeedControl)


# 小车左转
def left():
    pwm_ENA.ChangeDutyCycle(CarSpeedControl)
    pwm_ENB.ChangeDutyCycle(CarSpeedControl)
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    pwm_ENA.start(CarSpeedControl)
    pwm_ENB.start(CarSpeedControl)


# 小车右转
def right():
    pwm_ENA.ChangeDutyCycle(CarSpeedControl)
    pwm_ENB.ChangeDutyCycle(CarSpeedControl)
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.LOW)
    pwm_ENA.start(CarSpeedControl)
    pwm_ENB.start(CarSpeedControl)


# 小车原地左转
def spin_left():
    pwm_ENA.ChangeDutyCycle(CarSpeedControl)
    pwm_ENB.ChangeDutyCycle(CarSpeedControl)
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    pwm_ENA.start(CarSpeedControl)
    pwm_ENB.start(CarSpeedControl)


# 小车原地右转
def spin_right():
    pwm_ENA.ChangeDutyCycle(CarSpeedControl)
    pwm_ENB.ChangeDutyCycle(CarSpeedControl)
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)
    pwm_ENA.start(CarSpeedControl)
    pwm_ENB.start(CarSpeedControl)


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


# 前舵机旋转到指定角度
def frontservo_appointed_detection(pos):
    pulsewidth = (pos * 11) + 500
    GPIO.output(FrontServoPin, GPIO.HIGH)
    time.sleep(pulsewidth / 1000000.0)
    GPIO.output(FrontServoPin, GPIO.LOW)
    time.sleep(20.0 / 1000 - pulsewidth / 1000000.0)
    global nowfrontPos
    nowfrontPos = pos


def leftrightservo_appointed_detection(pos):
    pulsewidth = (pos * 11) + 500
    GPIO.output(ServoLeftRightPin, GPIO.HIGH)
    time.sleep(pulsewidth / 1000000.0)
    GPIO.output(ServoLeftRightPin, GPIO.LOW)
    time.sleep(20.0 / 1000 - pulsewidth / 1000000.0)
    global nowfrontPos
    nowfrontPos = pos


# 摄像头舵机上下旋转到指定角度
def updownservo_appointed_detection(pos):
    pulsewidth = (pos * 11) + 500
    GPIO.output(ServoUpDownPin, GPIO.HIGH)
    time.sleep(pulsewidth / 1000000.0)
    GPIO.output(ServoUpDownPin, GPIO.LOW)
    time.sleep(20.0 / 1000 - pulsewidth / 1000000.0)
    global nowfrontPos
    nowfrontPos = pos


def servo_init():
    servoinitpos = 90
    for i in range(18):
        frontservo_appointed_detection(servoinitpos)
        time.sleep(0.02)
        updownservo_appointed_detection(servoinitpos)
        time.sleep(0.02)
        leftrightservo_appointed_detection(servoinitpos)
        time.sleep(0.02)
        #  pwm_FrontServo.ChangeDutyCycle(0)  # 归零信号
    # pwm_LeftRightServo.ChangeDutyCycle(0)  # 归零信号
    # 0pwm_UpDownServo.ChangeDutyCycle(0)  # 归零信号


def auto():
    # init()
    #  servo_init()
    taxishu = 0.008
    FindNum = 0

    while FindNum == 0:
        distance = []
        temperature = []
        angle = []
        for i in range(7):
            for ii in range(9):
                frontservo_appointed_detection(i * 30)
                time.sleep(0.01)
            time.sleep(0.8)
            distance.append(Distance_test())
            t, k = tcam()
            temperature.append(t)
            k = int((k - 15.5) / 31 * 55)
            angle.append(k)
            # 正前方为0,右侧为负数,左为正
        for i in range(18):
            frontservo_appointed_detection(90)
            time.sleep(0.02)
        print(distance)
        print(temperature)
        print(angle)

        index = temperature.index(max(temperature))
        target_angle = angle[index] + index * 30
        print(index)
        print(target_angle)

        # 温度过高，找到火源
        if temperature[index] > 100:
            FindNum = FindNum + 1
            lat, lon, alt = GetGPS()
            print("-- Lat: %s -- Lon: %s -- Altitude: %s" % (lat, lon, alt))
            for i in range(3):
                servo_control_color()
            break

        if target_angle <= 90:
            # 目标在右
            needtime = (90 - target_angle) * taxishu
            spin_right()
            time.sleep(needtime)
            brake()
        elif target_angle > 90:
            # 目标在左
            needtime = (target_angle - 90) * taxishu
            spin_left()
            time.sleep(needtime)
            brake()

        if distance[index] > 60:
            run()
            time.sleep(2)
            brake()
        elif distance[index] < 60 and distance[index] > 40 or temperature[index] > 35:
            run()
            time.sleep(1)
            print("快了")
            brake()
        elif (distance[index] < 50 or distance[min(index + 1, 6)] < 50 or distance[max(0, index - 1)] < 50) and (
                temperature[index] < 38):
            print('避障')
            left()
            time.sleep(1)
            brake()
            time.sleep(0.2)
            run()
            time.sleep(1.5)
            brake()
            time.sleep(0.2)
            right()
            time.sleep(2)
            brake()


# 前舵机向左
def front_servo0():
    for i in range(18):
        frontservo_appointed_detection(0)
        time.sleep(0.02)


def front_servo45():
    for i in range(18):
        frontservo_appointed_detection(45)
        time.sleep(0.02)


def front_servo90():
    for i in range(18):
        frontservo_appointed_detection(90)
        time.sleep(0.02)


def front_servo135():
    for i in range(18):
        frontservo_appointed_detection(135)
        time.sleep(0.02)


def front_servo180():
    for i in range(18):
        frontservo_appointed_detection(180)
        time.sleep(0.02)

    # 摄像头舵机左右旋转到指定角度



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
        elif (len(recv_data) == 1) and (recv_data.decode('gbk')[0] == 'a'):
            with eventlet.Timeout(1, False):
                left()
        elif (len(recv_data) == 1) and (recv_data.decode('gbk')[0] == 'd'):
            with eventlet.Timeout(1, False):
                right()
        elif (len(recv_data) == 1) and (recv_data.decode('gbk')[0] == 'x'):
            with eventlet.Timeout(1, False):
                brake()
        elif (len(recv_data) == 1) and (recv_data.decode('gbk')[0] == 'e'):
            with eventlet.Timeout(1, False):
                spin_right()
        elif (len(recv_data) == 1) and (recv_data.decode('gbk')[0] == 'q'):
            with eventlet.Timeout(1, False):
                spin_left()
        elif (len(recv_data) == 1) and (recv_data.decode('gbk')[0] == 'y'):
            with eventlet.Timeout(1, False):
                front_servo0()
        elif (len(recv_data) == 1) and (recv_data.decode('gbk')[0] == 'u'):
            with eventlet.Timeout(1, False):
                front_servo45()
        elif (len(recv_data) == 1) and (recv_data.decode('gbk')[0] == 'i'):
            with eventlet.Timeout(1, False):
                front_servo90()
        elif (len(recv_data) == 1) and (recv_data.decode('gbk')[0] == 'o'):
            with eventlet.Timeout(1, False):
                front_servo135()
        elif (len(recv_data) == 1) and (recv_data.decode('gbk')[0] == 'p'):
            with eventlet.Timeout(1, False):
                front_servo180()
        elif (len(recv_data) == 1) and (recv_data.decode('gbk')[0] == 'n'):
            auto()
        # # else:
        # wiringpi.digitalWrite(0,0)
        # if len(recv_data) > 1:
        # wiringpi.digitalWrite(0,0)

        print('recv: %s' % recv_data.decode('gbk'))


def main():
    init()
    servo_init()
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
    my_addr = ('192.168.183.171', 8888) #che
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


        # 每当来新的客户端连接，创建子进程，由子进程和客户端通信
        process_do_service = Process(target=do_service, args=(connect_socket,))
        process_do_service.start()

        # 父进程，关闭connect_socket
        connect_socket.close()

if __name__ == '__main__':
    main()