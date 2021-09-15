import socket
import time

import socket
import time
import serial
import pynmea2
from ctypes import *
import numpy as np

mlx90640 = cdll.LoadLibrary('./libmlx90640.so')
serialPort = serial.Serial("/dev/ttyUSB0", 9600, timeout=0.5)

def tcam():
    temp = (c_float * 768)()
    ptemp = pointer(temp)
    mlx90640.get_mlx90640_temp(ptemp)
    my_nparray = np.frombuffer(temp, dtype=np.float32)

    t = my_nparray.reshape((32, 24))

    # print(np.max(t))
    # print(np.argmax(t))
    return np.max(t)



def parseGPS(s):
    if s.find('GGA') > -1:
        msg = pynmea2.parse(s)
        print("Timestamp: %s -- Lat: %s %s -- Lon: %s %s -- Altitude: %s %s" % (
            msg.timestamp, msg.lat, msg.lat_dir, msg.lon, msg.lon_dir, msg.altitude, msg.altitude_units))

def main():
    print("客户端开启")
    # 套接字接口
    mySocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    # 设置ip和端口
    host = '192.168.150.1'
    port = 4445

    try:
        mySocket.connect((host, port))  ##连接到服务器
        print("连接到服务器")
    except:  ##连接不成功，运行最初的ip
        print('连接不成功')

    while True:
        # 发送消息
        msg = tcam()
        t = int(msg)
        tt = str(t)
        # 编码发送
        mySocket.send(tt.encode("utf-8"))
        print("发送完成")

        time.sleep(0.5)

        if msg == "over":
            mySocket.close()
            print("程序结束\n")
            exit()
            break
    print("程序结束\n")
if __name__ == '__main__':
    main()

