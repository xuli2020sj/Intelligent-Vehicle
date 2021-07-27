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
# 小车速度变量
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


# 小车前进
def run():
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    # 启动PWM设置占空比为100（0--100）
    pwm_ENA.start(100)
    pwm_ENB.start(100)
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
    pulsewidth = (pos * 11) + 500
    GPIO.output(FrontServoPin, GPIO.HIGH)
    time.sleep(pulsewidth / 1000000.0)
    GPIO.output(FrontServoPin, GPIO.LOW)
    time.sleep(20.0 / 1000 - pulsewidth / 1000000.0)
    global nowfrontPos
    nowfrontPos = pos
    # time.sleep(0.02)
    # for i in range(18):
    #     pwm_FrontServo.start(2.5 + 10 * pos / 180)
    #     time.sleep(0.02)  # 等待20ms周期结束
    #     # pwm_FrontServo.ChangeDutyCycle(0)  # 归零信号


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


def leftrightservo_appointed_detection(pos):
    for i in range(18):
        pwm_LeftRightServo.ChangeDutyCycle(2.5 + 10 * pos / 180)
        time.sleep(0.2)  # 等待20ms周期结束
        # pwm_LeftRightServo.ChangeDutyCycle(0)	#归零信号


# 摄像头舵机上下旋转到指定角度
def updownservo_appointed_detection(pos):
    for i in range(18):
        pwm_UpDownServo.ChangeDutyCycle(2.5 + 10 * pos / 180)
        time.sleep(0.2)  # 等待20ms周期结束
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


def main():
    init()
    servo_init()
    taxishu=0.005
    FindNum=0
    global FindNum


    while FindNum==0:
        distance=[]
        temperature=[]
        angle=[]
        for i in range(7):
            frontservo_appointed_detection(i*30)
            time.sleep(0.8)
            distance.append(Distance_test())
            temperature.append(hongwai_temp())
            angle.append(hongwai_angle())
            # %正前方为0,右侧为负数,左为正

        index=temperature.index(max(temperature))
        target_angle=angle(index)+index*30

        if temperature(index)>120:
            FindNum = FindNum+1
            break

        if target_angle<=90 :
            needtime=(90-target_angle)*taxishu
            spin_right()
            time.sleep(needtime)
            brake()
        elif target_angle>90:
            needtime=(target_angle-90)*taxishu
            spin_left()
            time.sleep(needtime)
            brake()

        if distance(index)>30
            run()
            time.sleep(2)
            brake()
        elif distance(index)<30 and distance(index)>20
            run()
            time.sleep(1)
            brake()





if __name__ == "__main__":
    main()