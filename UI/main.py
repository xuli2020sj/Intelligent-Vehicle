# coding=UTF-8
import os
import sys
import cv2
import PyQt5
from PyQt5 import QtCore
from PyQt5.QtCore import QCoreApplication, QUrl, pyqtSignal, QObject
from PyQt5.QtWebEngineWidgets import QWebEngineView
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QMessageBox, QWidget
from PyQt5.uic import loadUiType
from ipywidgets.widgets import widget
import motion
from socket import *
from multiprocessing import Process
from socket import *
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

Ui_MainWindow = loadUiType("main.ui")[0]


class MyTypeSignal(QObject):
    # 定义一个信号
    sendmsg = pyqtSignal(object)

    # 调用run来实现触发
    def run(self):
        self.sendmsg.emit('Hello PyQt5')  # 给槽传递一个参数


class MySlot(QObject):
    # 槽函数
    def get(self, msg):
        print('信息：' + msg)

class MainWindow(QMainWindow, Ui_MainWindow):

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
        # 启动寻源
        self.start.clicked.connect(self.appendText)
        # 停止寻源
        self.stop.clicked.connect(self.stopText)
        # 车辆返航
#        self.return_to_base.clicked(self.appendText3)
        # 绑定快捷键
        self.upbtn.setShortcut('w')
        self.upbtn.released.connect(self.forwardText)
        self.downbtn.setShortcut('s')
        self.downbtn.clicked.connect(self.backText)
        self.rightbtn.setShortcut('d')
        self.rightbtn.clicked.connect(self.rightText)
        self.leftbtn.setShortcut('a')
        self.leftbtn.clicked.connect(self.leftText)
        self.stop.setShortcut('x')
        self.stop.pressed.connect(self.stopText)

    # 状态信息显示
    def appendText(self):
        self.textBrowser.append("自动寻源已启动")

    def stopText(self):
        self.textBrowser.append("小车已停止")

    def appendText3(self):
        self.textBrowser.append("车辆正在返 航")

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

    # def showMsg():
    #     QMessageBox.information(self.widget, '信息提示框', 'cdse')


    def connect(self):
        # 1.创建socket
        client_socket = socket(AF_INET, SOCK_STREAM)

        # 2.指定服务器的地址和端口号
        server_addr = ('192.168.3.24', 8888)
        client_socket.connect(server_addr)

        print('connect %s success' % str(server_addr))

        while True:
            # 3.给用户提示，让用户输入要检索的资料
            send_data = input('>>')
            # 退出
            if send_data == 'quit':
                break
            # 向服务器请求数据
            client_socket.send(send_data.encode())

        client_socket.close()


def qtmain():
    QtCore.QCoreApplication.setAttribute(QtCore.Qt.AA_EnableHighDpiScaling)
    app = QApplication(sys.argv)
    mw = MainWindow()
    mw.show()
    # 自定义信号与槽
    send = MyTypeSignal()
    slot = MySlot()
    send.sendmsg.connect(slot.get)
    send.run()#发送信号

    sys.exit(app.exec_())

if __name__ == '__main__':
    qtmain()

