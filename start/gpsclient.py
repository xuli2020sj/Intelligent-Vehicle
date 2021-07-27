import socket
import time
import serial
import pynmea2
from ctypes import *
import numpy as np

mlx90640 = cdll.LoadLibrary('./libmlx90640.so')

def tcam():
    temp = (c_float * 768)()
    ptemp = pointer(temp)
    mlx90640.get_mlx90640_temp(ptemp)
    my_nparray = np.frombuffer(temp, dtype=np.float32)

    t = my_nparray.reshape((32, 24))

    # print(np.max(t))
    # print(np.argmax(t))
    return np.max(t)

serialPort = serial.Serial("/dev/ttyUSB0", 9600, timeout=0.5)

def parseGPS(s):
    if s.find('GGA') > -1:
        msg = pynmea2.parse(s)
        print("Timestamp: %s -- Lat: %s %s -- Lon: %s %s -- Altitude: %s %s" % (
            msg.timestamp, msg.lat, msg.lat_dir, msg.lon, msg.lon_dir, msg.altitude, msg.altitude_units))


def main():
    # 1.创建socket
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    # 2.指定服务器的地址和端口号
    server_addr = ('192.168.146.1', 7777)
    client_socket.connect(server_addr)

    print('connect %s success' % str(server_addr))

    while True:

        # send_data = str(int(tcam()))
        # print(send_data)
        send_data = "fuck"
        if send_data == 'quit':
            break
        # 向服务器请求数据
        client_socket.send(send_data.encode())
        time.sleep(0.5)

    client_socket.close()


if __name__ == "__main__":
    main()
