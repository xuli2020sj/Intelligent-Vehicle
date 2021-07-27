from multiprocessing import Process
from socket import *
# import wiringpi

import RPi.GPIO as GPIO
import time
import string
import threading
import timeout_decorator
import eventlet
import time

eventlet.monkey_patch()

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
ServoPin = 23

# 红外避障引脚定义
AvoidSensorLeft = 12
AvoidSensorRight = 17

# 设置GPIO口为BCM编码方式
GPIO.setmode(GPIO.BCM)

# 忽略警告信息
GPIO.setwarnings(False)


# 电机引脚初始化为输出模式
# 按键引脚初始化为输入模式
# 超声波,RGB三色灯,舵机引脚初始化
# 红外避障引脚初始化
def init():
    global pwm_ENA
    global pwm_ENB
    global pwm_servo
    GPIO.setup(ENA, GPIO.OUT, initial=GPIO.HIGH)
    GPIO.setup(IN1, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(IN2, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(ENB, GPIO.OUT, initial=GPIO.HIGH)
    GPIO.setup(IN3, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(IN4, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(key, GPIO.IN)
    GPIO.setup(EchoPin, GPIO.IN)
    GPIO.setup(TrigPin, GPIO.OUT)
    GPIO.setup(LED_R, GPIO.OUT)
    GPIO.setup(LED_G, GPIO.OUT)
    GPIO.setup(LED_B, GPIO.OUT)
    GPIO.setup(ServoPin, GPIO.OUT)
    GPIO.setup(AvoidSensorLeft, GPIO.IN)
    GPIO.setup(AvoidSensorRight, GPIO.IN)
    # 设置pwm引脚和频率为2000hz
    pwm_ENA = GPIO.PWM(ENA, 2000)
    pwm_ENB = GPIO.PWM(ENB, 2000)
    pwm_ENA.start(0)
    pwm_ENB.start(0)
    # 设置舵机的频率和起始占空比
    pwm_servo = GPIO.PWM(ServoPin, 50)
    pwm_servo.start(0)


# 小车前进
def run(leftspeed, rightspeed):
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    pwm_ENA.ChangeDutyCycle(leftspeed)
    pwm_ENB.ChangeDutyCycle(rightspeed)


# 小车后退
def back(leftspeed, rightspeed):
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)
    pwm_ENA.ChangeDutyCycle(leftspeed)
    pwm_ENB.ChangeDutyCycle(rightspeed)


# 小车左转
def left(leftspeed, rightspeed):
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    pwm_ENA.ChangeDutyCycle(leftspeed)
    pwm_ENB.ChangeDutyCycle(rightspeed)


# 小车右转
def right(leftspeed, rightspeed):
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.LOW)
    pwm_ENA.ChangeDutyCycle(leftspeed)
    pwm_ENB.ChangeDutyCycle(rightspeed)


# 小车原地左转
def spin_left(leftspeed, rightspeed):
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    pwm_ENA.ChangeDutyCycle(leftspeed)
    pwm_ENB.ChangeDutyCycle(rightspeed)


# 小车原地右转
def spin_right(leftspeed, rightspeed):
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)
    pwm_ENA.ChangeDutyCycle(leftspeed)
    pwm_ENB.ChangeDutyCycle(rightspeed)


# 小车停止
def brake():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.LOW)


# 按键检测
def key_scan():
    while GPIO.input(key):
        pass
    while not GPIO.input(key):
        time.sleep(0.01)
        if not GPIO.input(key):
            time.sleep(0.01)
            while not GPIO.input(key):
                pass


# 超声波函数
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
    print("distance is %d " % (((t2 - t1) * 340 / 2) * 100))
    time.sleep(0.01)
    return ((t2 - t1) * 340 / 2) * 100


# 舵机旋转到指定角度
def servo_appointed_detection(pos):
    for i in range(18):
        pwm_servo.ChangeDutyCycle(2.5 + 10 * pos / 180)


# 舵机旋转超声波测距避障，led根据车的状态显示相应的颜色
def servo_color_carstate():
    # 开红灯
    GPIO.output(LED_R, GPIO.HIGH)
    GPIO.output(LED_G, GPIO.LOW)
    GPIO.output(LED_B, GPIO.LOW)
    back(20, 20)
    time.sleep(0.08)
    brake()

    # 舵机旋转到0度，即右侧，测距
    servo_appointed_detection(0)
    time.sleep(0.8)
    rightdistance = Distance_test()

    # 舵机旋转到180度，即左侧，测距
    servo_appointed_detection(180)
    time.sleep(0.8)
    leftdistance = Distance_test()

    # 舵机旋转到90度，即前方，测距
    servo_appointed_detection(90)
    time.sleep(0.8)
    frontdistance = Distance_test()

    if leftdistance < 30 and rightdistance < 30 and frontdistance < 30:
        # 亮品红色，掉头
        GPIO.output(LED_R, GPIO.HIGH)
        GPIO.output(LED_G, GPIO.LOW)
        GPIO.output(LED_B, GPIO.HIGH)
        spin_right(85, 85)
        time.sleep(0.58)
    elif leftdistance >= rightdistance:
        # 亮蓝色
        GPIO.output(LED_R, GPIO.LOW)
        GPIO.output(LED_G, GPIO.LOW)
        GPIO.output(LED_B, GPIO.HIGH)
        spin_left(85, 85)
        time.sleep(0.28)
    elif leftdistance <= rightdistance:
        # 亮品红色，向右转
        GPIO.output(LED_R, GPIO.HIGH)
        GPIO.output(LED_G, GPIO.LOW)
        GPIO.output(LED_B, GPIO.HIGH)
        spin_right(85, 85)
        time.sleep(0.28)


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
            with eventlet.Timeout(0.1, False):
                run(100, 100)
        elif (len(recv_data) == 1) and (recv_data.decode('gbk')[0] == 's'):
            with eventlet.Timeout(0.1, False):
                back(100, 100)
        elif (len(recv_data) == 1) and (recv_data.decode('gbk')[0] == 'a'):
            with eventlet.Timeout(0.1, False):
                left(100, 100)
        elif (len(recv_data) == 1) and (recv_data.decode('gbk')[0] == 'd'):
            with eventlet.Timeout(0.1, False):
                right(100, 100)
        elif (len(recv_data) == 1) and (recv_data.decode('gbk')[0] == 'x'):
            with eventlet.Timeout(0.1, False):
                brake()
        elif (len(recv_data) == 1) and (recv_data.decode('gbk')[0] == 'e'):
            with eventlet.Timeout(0.1, False):
                spin_right(100, 100)
        elif (len(recv_data) == 1) and (recv_data.decode('gbk')[0] == 'q'):
            with eventlet.Timeout(0.1, False):
                spin_left(100, 100)
        elif (len(recv_data) == 1) and (recv_data.decode('gbk')[0] == 'y'):
            with eventlet.Timeout(0.1, False):
                servo_appointed_detection(0)
        elif (len(recv_data) == 1) and (recv_data.decode('gbk')[0] == 'u'):
            with eventlet.Timeout(0.1, False):
                servo_appointed_detection(45)
        elif (len(recv_data) == 1) and (recv_data.decode('gbk')[0] == 'i'):
            with eventlet.Timeout(0.1, False):
                servo_appointed_detection(90)
        elif (len(recv_data) == 1) and (recv_data.decode('gbk')[0] == 'o'):
            with eventlet.Timeout(0.1, False):
                servo_appointed_detection(135)
        elif (len(recv_data) == 1) and (recv_data.decode('gbk')[0] == 'p'):
            with eventlet.Timeout(0.1, False):
                servo_appointed_detection(180)
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
