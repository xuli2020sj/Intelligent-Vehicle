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
        self.resize(1200, 900)

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
        self.stop.clicked.connect(self.appendText2)
        # 车辆返航
        self.return_to_base.clicked(self.appendText3)
        # 绑定快捷键
        self.upbtn.setShortcut('w')
        self.downbtn.setShortcut('s')
        self.rightbtn.setShortcut('d')
        self.leftbtn.setShortcut('a')

    # 状态信息显示
    def appendText(self):
        self.textBrowser.append("自动寻源已启动")

    def appendText2(self):
        self.textBrowser.append("自动寻源已停止")

    def appendText3(self):
        self.textBrowser.append("车辆正在返航")

    def appenTextSpeed(self):
        self.textBrowser.append("")

    def signalCall1(self):
        print("signal1 emit")

    # def showMsg():
    #     QMessageBox.information(self.widget, '信息提示框', 'cdse')


if __name__ == '__main__':
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
