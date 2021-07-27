import socket
import time

print("客户端开启")
# 套接字接口
mySocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
# 设置ip和端口
host = '192.168.146.1'
port = 2222

try:
    mySocket.connect((host, port))  ##连接到服务器
    print("连接到服务器")
except:  ##连接不成功，运行最初的ip
    print('连接不成功')

while True:
    # 发送消息
    msg = '9'
    # 编码发送
    mySocket.send(msg.encode("utf-8"))
    print("发送完成")

    time.sleep(10)

    if msg == "over":
        mySocket.close()
        print("程序结束\n")
        exit()
        break
print("程序结束\n")


