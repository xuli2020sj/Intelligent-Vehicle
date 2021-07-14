# coding=UTF-8
import os
import sys
import cv2
import PyQt5
from PyQt5 import QtCore
from PyQt5.QtCore import QCoreApplication, QUrl
from PyQt5.QtWebEngineWidgets import QWebEngineView
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout
from PyQt5.uic import loadUiType

Ui_MainWindow = loadUiType("main.ui")[0]


class MainWindow(QMainWindow, Ui_MainWindow):
    def __init__(self, parent=None):
        super(MainWindow, self).__init__(parent)
        self.setupUi(self)
        self.resize(800, 600)

        self.browser=QWebEngineView()
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
            mymap.on('click', function(e) {
                var lng = e.lnglat.getLng();
                var lat = e.lnglat.getLat();
                infoWindow.close();
            }
            );
        </script>
        </body>
        </html>''')
        # 设置垂直布局器
        layout = QVBoxLayout()
        self.setLayout(layout)
        self.gridLayout_2.addWidget(self.browser)

    def loadPage(self):
        with open('gdmap.html', 'r', encoding="utf-8") as f:
            html = f.read()
            self.browser.setHtml(html)


if __name__ == '__main__':
    QtCore.QCoreApplication.setAttribute(QtCore.Qt.AA_EnableHighDpiScaling)
    app = QApplication(sys.argv)
    mw = MainWindow()

    mw.show()
    sys.exit(app.exec_())
