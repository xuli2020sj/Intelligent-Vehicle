import socket
import time


def main():
    # 1.创建socket
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    # 2.指定服务器的地址和端口号
    server_addr = ('192.168.146.1', 7777)
    client_socket.connect(server_addr)

    print('connect %s success' % str(server_addr))

    while True:

        send_data = "fuck"

        if send_data == 'quit':
            break
        # 向服务器请求数据
        client_socket.send(send_data.encode())
        time.sleep(0.5)

    client_socket.close()


if __name__ == "__main__":
    main()