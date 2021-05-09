import sys
import cv2
import PyQt5
from PyQt5.QtWidgets import QApplication, QMainWindow
from PyQt5.uic import loadUiType

Ui_MainWindow = loadUiType("main.ui")[0]


class MainWindow(QMainWindow, Ui_MainWindow):
    def __init__(self, parent=None):
        super(MainWindow, self).__init__(parent)
        self.setupUi(self)

    def auto(self):
        print("无人驾驶已启动")

if __name__ == '__main__':
    app = QApplication(sys.argv)
    mw = MainWindow()
    mw.show()
    sys.exit(app.exec_())
