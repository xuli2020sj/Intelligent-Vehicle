# -*- coding: utf-8 -*-
"""
Created on Sat Jul 10 21:09:10 2021

@author: qq735
"""
import socket
 
def main():
    # 1.创建socket
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

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
 
if __name__ == "__main__":
    print("hi")
    main()
