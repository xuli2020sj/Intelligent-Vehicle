# coding=UTF-8
import os
import struct
import sys
import time

import cv2
import PyQt5
import numpy
import numpy as np
from PyQt5 import QtCore, QtWidgets, QtGui
from PyQt5.QtCore import QCoreApplication, QUrl, pyqtSignal, QObject, QThread, pyqtSlot, Qt, QTimer
from PyQt5.QtGui import QPixmap
from PyQt5.QtWebEngineWidgets import QWebEngineView
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QMessageBox, QWidget
from PyQt5.uic import loadUiType
from ipywidgets.widgets import widget
import motion
import socket
from multiprocessing import Process

import threading
# import wiringpi
import threading

# 小车电机引脚定义
IN1 = 20
IN2 = 21
IN3 = 19
IN4 = 26
ENA = 16
ENB = 13
CarSpeedControl = 1000
temperature = 30

# 加载qt文件
Ui_MainWindow = loadUiType("main.ui")[0]


class MainWindow(QMainWindow, Ui_MainWindow):
    motionSin = pyqtSignal(str)

    def __init__(self, parent=None):
        super(MainWindow, self).__init__(parent)
        self.setupUi(self)
        self.resize(950, 650)

        self.browser = QWebEngineView()
        self.browser.setHtml('''<!doctype html>
        <html>
        <head>
            <meta charset="utf-8">
            <meta http-equiv="X-UA-Compatible" content="IE=edge">
            <meta name="viewport" content="initial-scale=1.0, user-scalable=no, width=device-width">
            <link rel="stylesheet" href="https://a.amap.com/jsapi_demos/static/demo-center/css/demo-center.css" />
            <title>地图显示</title>
            <style>
                html,
                body,
                #container {
                  width: 100%;
                  height: 100%;
                }
            </style>
        </head>
        <body>
        <div id="container"></div>
        <!-- 加载地图JSAPI脚本 -->
        <script src="https://webapi.amap.com/maps?v=1.4.15&key=7ac71cf4a748f108b28b588d15de5948"></script>
        <script>
            var map = new AMap.Map('container', {
                resizeEnable: true, //是否监控地图容器尺寸变化
                zoom:16, //初始化地图层级
                center: [121.447477,31.025758] //初始化地图中心点
            }
            );
        </script>
        </body>
        </html>''')
        # 设置垂直布局器
        layout = QVBoxLayout()
        self.setLayout(layout)
        self.gridLayout_2.addWidget(self.browser)

        # 车辆返航
        #        self.return_to_base.clicked(self.appendText3)

        # 绑定快捷键
        self.upbtn.setShortcut('w')
        self.upbtn.clicked.connect(self.forwardText)
        self.upbtn.clicked.connect(self.forwardc)

        self.downbtn.setShortcut('s')
        self.downbtn.clicked.connect(self.backText)
        self.downbtn.clicked.connect(self.backc)

        self.rightbtn.setShortcut('d')
        self.rightbtn.clicked.connect(self.rightText)
        self.rightbtn.clicked.connect(self.rightc)

        self.leftbtn.setShortcut('a')
        self.leftbtn.clicked.connect(self.leftText)
        self.leftbtn.clicked.connect(self.leftc)

        self.srightbtn.setShortcut('e')
        # self.srightbtn.clicked.connect(self.srightText)
        self.srightbtn.clicked.connect(self.srightc)

        self.sleftbtn.setShortcut('q')
        # self.sleftbtn.clicked.connect(self.sleftText)
        self.sleftbtn.clicked.connect(self.sleftc)

        self.stop.setShortcut('x')
        self.stop.clicked.connect(self.stopText)
        self.stop.clicked.connect(self.stopc)

        self.start.setShortcut('n')
        self.start.clicked.connect(self.startText)
        self.start.clicked.connect(self.startc)

        self.motionSin.connect(self.ccmd)

        # self.label_show_camera = QtWidgets.QLabel()  # 定义显示视频的Label
        # self.label_show_camera.setFixedSize(641, 481)  # 给显示视频的Label设置大小为641x481

        self.camthread = CamThread()  # 实例化任务线程类
        self.connect.clicked.connect(self.camStart)

        self.tcpthread = TcpThread()  # 实例化任务线程类
        self.connect.clicked.connect(self.TcpStart)

        self.tcamthread = thermalCamThread()  # 实例化任务线程类
        self.connect.clicked.connect(self.TcaStart)

        time = QTimer(self)
        time.setInterval(500)
        time.timeout.connect(self.refresh)
        time.start()
        self.tlcd.display(25)

        self.jdlcd.display(121.447477)
        self.wdlcd.display(31.025758)

    def refresh(self):
        self.ttlcd.display(temperature)

    def forwardc(self):
        self.motionSin.emit("w")

    def backc(self):
        self.motionSin.emit("s")

    def rightc(self):
        self.motionSin.emit("d")

    def leftc(self):
        self.motionSin.emit("a")

    def srightc(self):
        self.motionSin.emit("e")

    def sleftc(self):
        self.motionSin.emit("q")

    def stopc(self):
        self.motionSin.emit("x")

    def startc(self):
        self.motionSin.emit("n")

    def ccmd(self, text):
        # print(text)
        self.tcpthread.setMesg(text)

    # 启动摄像头
    def camStart(self):
        self.camthread.start()

    # 启动TCP
    def TcpStart(self):
        self.tcpthread.start()

    def TcaStart(self):
        self.tcamthread.start()

    # 状态信息显示
    def startText(self):
        self.textBrowser.append("自动寻源已启动")

    def stopText(self):
        self.textBrowser.append("小车已停止")

    def appendText3(self):
        self.textBrowser.append("车辆正在返航")

    def appenTextSpeed(self):
        self.textBrowser.append("")

    def signalCall1(self):
        print("signal1 emit")

    # 状态信息显示
    def forwardText(self):
        self.textBrowser.append("小车前进")

    def backText(self):
        self.textBrowser.append("小车后退")

    def rightText(self):
        self.textBrowser.append("小车右转")

    def leftText(self):
        self.textBrowser.append("小车左转")

    # @pyqtSlot(np.ndarray)
    # def update_image(self, cv_img):
    #     """Updates the image_label with a new opencv image"""
    #     qt_img = self.convert_cv_qt(cv_img)
    #     self.image_label.setPixmap(qt_img)
    #
    # def convert_cv_qt(self, cv_img):
    #     """Convert from an opencv image to QPixmap"""
    #     rgb_image = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
    #     h, w, ch = rgb_image.shape
    #     bytes_per_line = ch * w
    #     convert_to_Qt_format = QtGui.QImage(rgb_image.data, w, h, bytes_per_line, QtGui.QImage.Format_RGB888)
    #     p = convert_to_Qt_format.scaled(self.disply_width, self.display_height, Qt.KeepAspectRatio)
    #     return QPixmap.fromImage(p)


class CamThread(QThread):  # 建立一个任务线程类
    signal = pyqtSignal(str)  # 设置触发信号传递的参数数据类型,这里是字符串

    def __init__(self):
        super(CamThread, self).__init__()


    def run(self):  # 在启动线程后任务从这个函数里面开始执行
        HOST = '192.168.146.1'
        PORT = 9999
        buffSize = 65535

        server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # 创建socket对象
        server.bind((HOST, PORT))
        print('now waiting for frames...')
        while True:
            data, address = server.recvfrom(buffSize)  # 先接收的是字节长度
            if len(data) == 1 and data[0] == 1:  # 如果收到关闭消息则停止程序
                server.close()
                cv2.destroyAllWindows()
                exit()
            if len(data) != 4:  # 进行简单的校验，长度值是int类型，占四个字节
                length = 0
            else:
                length = struct.unpack('i', data)[0]  # 长度值
            data, address = server.recvfrom(buffSize)  # 接收编码图像数据
            if length != len(data):  # 进行简单的校验
                continue
            data = numpy.array(bytearray(data))  # 格式转换
            imgdecode = cv2.imdecode(data, 1)  # 解码
            print('have received one frame')
            cv2.imshow('frames', imgdecode)  # 窗口显示
            if cv2.waitKey(1) == 27:  # 按下“ESC”退出
                break
        server.close()
        cv2.destroyAllWindows()


class thermalCamThread(QThread):  # 建立一个任务线程类
    signal = pyqtSignal(str)  # 设置触发信号传递的参数数据类型,这里是字符串

    def __init__(self):
        super(thermalCamThread, self).__init__()

    def run(self):  # 在启动线程后任务从这个函数里面开始执行
        HOST = '192.168.146.1'
        PORT = 7777
        buffSize = 65535

        server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # 创建socket对象
        server.bind((HOST, PORT))
        print('now waiting for data...')
        while True:
            data, address = server.recvfrom(buffSize)  # 先接收的是字节长度
            print(data)

            if data == 27:  # 按下“ESC”退出
                break
        server.close()



class TcpThread(QThread):  # 建立一个任务线程类
    signal = pyqtSignal(str)  # 设置触发信号传递的参数数据类型,这里是字符串
    mesg = ""

    def __init__(self):
        super(TcpThread, self).__init__()

    def setMesg(self, str):
        print(str)
        self.mesg = str

    def run(self):  # 在启动线程后任务从这个函数里面开始执行
        # 1.创建socket
        client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        # 2.指定服务器的地址和端口号
        server_addr = ('192.168.146.205', 8888)
        client_socket.connect(server_addr)

        print('connect %s success' % str(server_addr))

        mesg2 = self.mesg
        while True:
            # # 3.给用户提示，让用户输入要检索的资料
            # send_data = input('>>')
            # # 退出
            if self.mesg == 'quit':
                break
            # # 向服务器请求数据
            # client_socket.send(send_data.encode())
            if self.mesg != mesg2:
                client_socket.send(self.mesg.encode())
                time.sleep(0.1)
                mesg2 = self.mesg

        client_socket.close()

        # for i in range(10):
        #     print("i")
        #     # self.signal.emit(str(i))  # 任务线程发射信号用于与图形化界面进行交互
        #     time.sleep(1)


def qtmain():
    QtCore.QCoreApplication.setAttribute(QtCore.Qt.AA_EnableHighDpiScaling)
    app = QApplication(sys.argv)
    mw = MainWindow()
    mw.show()
    # 自定义信号与槽
    # send = MyTypeSignal()
    # slot = MySlot()
    # send.sendmsg.connect(slot.get)
    # send.run()  # 发送信号

    sys.exit(app.exec_())


if __name__ == '__main__':
    qtmain()
