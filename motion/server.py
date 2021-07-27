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
# 小车速度变量
CarSpeedControl = 100
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
    # global pwm_ENA
    # global pwm_ENB
    # global delaytime
    # global CarSpeedControl
    # global pwm_FrontServo
    # global pwm_UpDownServo
    # global pwm_LeftRightServo
    #
    # GPIO.setup(ENA, GPIO.OUT, initial=GPIO.HIGH)
    # GPIO.setup(IN1, GPIO.OUT, initial=GPIO.LOW)
    # GPIO.setup(IN2, GPIO.OUT, initial=GPIO.LOW)
    # GPIO.setup(ENB, GPIO.OUT, initial=GPIO.HIGH)
    # GPIO.setup(IN3, GPIO.OUT, initial=GPIO.LOW)
    # GPIO.setup(IN4, GPIO.OUT, initial=GPIO.LOW)
    #
    # GPIO.setup(EchoPin, GPIO.IN)
    # GPIO.setup(TrigPin, GPIO.OUT)
    #
    # GPIO.setup(FrontServoPin, GPIO.OUT)
    # pwm_FrontServo = GPIO.PWM(FrontServoPin, 50)
    # pwm_FrontServo.start(0)
    #
    # GPIO.setup(ServoUpDownPin, GPIO.OUT)
    # GPIO.setup(ServoLeftRightPin, GPIO.OUT)
    # GPIO.setup(buzzer, GPIO.OUT, initial=GPIO.HIGH)
    #
    # GPIO.setup(LED_R, GPIO.OUT)
    # GPIO.setup(LED_G, GPIO.OUT)
    # GPIO.setup(LED_B, GPIO.OUT)
    #
    # # 设置pwm引脚和频率为2000hz
    # pwm_ENA = GPIO.PWM(ENA, 2000)
    # pwm_ENB = GPIO.PWM(ENB, 2000)
    # # pwm_ENA.start(0)
    # # pwm_ENB.start(0)
    #
    # pwm_FrontServo = GPIO.PWM(FrontServoPin, 50)
    # pwm_UpDownServo = GPIO.PWM(ServoUpDownPin, 50)
    # pwm_LeftRightServo = GPIO.PWM(ServoLeftRightPin, 50)
    # pwm_FrontServo.start(0)
    # pwm_UpDownServo.start(0)
    # pwm_LeftRightServo.start(0)
    global pwm_ENA
    global pwm_ENB
    global pwm_FrontServo
    global pwm_UpDownServo
    global pwm_LeftRightServo
    global pwm_rled
    global pwm_gled
    global pwm_bled
    GPIO.setup(ENA, GPIO.OUT, initial=GPIO.HIGH)
    GPIO.setup(IN1, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(IN2, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(ENB, GPIO.OUT, initial=GPIO.HIGH)
    GPIO.setup(IN3, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(IN4, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(buzzer, GPIO.OUT, initial=GPIO.HIGH)

    GPIO.setup(EchoPin, GPIO.IN)
    GPIO.setup(TrigPin, GPIO.OUT)
    GPIO.setup(LED_R, GPIO.OUT)
    GPIO.setup(LED_G, GPIO.OUT)
    GPIO.setup(LED_B, GPIO.OUT)
    GPIO.setup(FrontServoPin, GPIO.OUT)
    GPIO.setup(ServoUpDownPin, GPIO.OUT)
    GPIO.setup(ServoLeftRightPin, GPIO.OUT)
    GPIO.setup(AvoidSensorLeft, GPIO.IN)
    GPIO.setup(AvoidSensorRight, GPIO.IN)
    # 设置pwm引脚和频率为2000hz
    pwm_ENA = GPIO.PWM(ENA, 2000)
    pwm_ENB = GPIO.PWM(ENB, 2000)
    pwm_ENA.start(0)
    pwm_ENB.start(0)
    # 设置舵机的频率和起始占空比
    pwm_FrontServo = GPIO.PWM(FrontServoPin, 50)
    pwm_FrontServo.start(0)
    pwm_UpDownServo = GPIO.PWM(ServoUpDownPin, 50)
    pwm_LeftRightServo = GPIO.PWM(ServoLeftRightPin, 50)

    pwm_UpDownServo.start(0)
    pwm_LeftRightServo.start(0)
    pwm_rled = GPIO.PWM(LED_R, 1000)
    pwm_gled = GPIO.PWM(LED_G, 1000)
    pwm_bled = GPIO.PWM(LED_B, 1000)
    pwm_rled.start(0)
    pwm_gled.start(0)
    pwm_bled.start(0)

# 小车前进
def run():
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    # 启动PWM设置占空比为100（0--100）
    # pwm_ENA.start(100)
    # pwm_ENB.start(100)
    pwm_ENA.ChangeDutyCycle(CarSpeedControl)
    pwm_ENB.ChangeDutyCycle(CarSpeedControl)


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


# 前舵机旋转到指定角度
def frontservo_appointed_detection(pos):
        # pulsewidth = (pos * 11) + 500
        # GPIO.output(FrontServoPin, GPIO.HIGH)
        # time.sleep(pulsewidth / 1000000.0)
        # GPIO.output(FrontServoPin, GPIO.LOW)
        # time.sleep(20.0 / 1000 - pulsewidth / 1000000.0)
    pos = int(pos)
    for i in range(18):
        pwm_FrontServo.start(2.5 + 10 * pos / 180)
        time.sleep(0.02)  # 等待20ms周期结束
        # pwm_FrontServo.ChangeDutyCycle(0)  # 归零信号

# 前舵机向左
def front_servo_left():
    frontservo_appointed_detection(180)


# 前舵机向右
def front_servo_right():
    frontservo_appointed_detection(0)

    # 摄像头舵机左右旋转到指定角度
def leftrightservo_appointed_detection(pos):
    for i in range(1):
        pwm_LeftRightServo.ChangeDutyCycle(2.5 + 10 * pos / 180)
        time.sleep(0.02)  # 等待20ms周期结束
        # pwm_LeftRightServo.ChangeDutyCycle(0)	#归零信号

# 摄像头舵机上下旋转到指定角度
def updownservo_appointed_detection(pos):
    for i in range(1):
        pwm_UpDownServo.ChangeDutyCycle(2.5 + 10 * pos / 180)
        time.sleep(0.02)  # 等待20ms周期结束
        # pwm_UpDownServo.ChangeDutyCycle(0)	#归零信号

def servo_init():
    servoflag = 0
    servoinitpos = 90
    if servoflag != servoinitpos:
        frontservo_appointed_detection(servoinitpos)
        updownservo_appointed_detection(servoinitpos)
        leftrightservo_appointed_detection(servoinitpos)
        time.sleep(0.5)
        pwm_FrontServo.ChangeDutyCycle(0)  # 归零信号
        pwm_LeftRightServo.ChangeDutyCycle(0)  # 归零信号
        pwm_UpDownServo.ChangeDutyCycle(0)  # 归零信号


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
            front_servo_left()
            GPIO.output(LED_R, GPIO.LOW)
            GPIO.output(LED_G, GPIO.LOW)
            GPIO.output(LED_B, GPIO.HIGH)
            whistle()
        elif (len(recv_data) == 1) and (recv_data.decode('gbk')[0] == 'i'):
            front_servo_right()
            GPIO.output(LED_R, GPIO.HIGH)
            GPIO.output(LED_G, GPIO.LOW)
            GPIO.output(LED_B, GPIO.LOW)
        elif (len(recv_data) == 1) and (recv_data.decode('gbk')[0] == 'o'):
            updownservo_appointed_detection(50)
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
