
from multiprocessing import Process
from socket import *
def do_service(connect_socket):
    while True:
        recv_data = connect_socket.recv(1024)
        if len(recv_data) == 0:
            # 发送方关闭tcp的连接,recv()不会阻塞，而是直接返回''
            # print('client %s close' % str(client_addr))
            # s.getpeername()   s.getsockname()
            # wiringpi.digitalWrite(0,0)
            print('client %s close' % str(connect_socket.getpeername()))
            break

        print('recv: %s' % recv_data.decode('gbk'))
# 1.创建socket
listen_socket = socket(AF_INET, SOCK_STREAM)
# stream流式套接字,对应tcp

# 设置允许复用地址,当建立连接之后服务器先关闭，设置地址复用
# 设置socket层属性    复用地址，不用等2msl，    允许
listen_socket.setsockopt(SOL_SOCKET, SO_REUSEADDR, 1)

# 2.绑定端口
my_addr = ('192.168.146.1', 7777)
listen_socket.bind(my_addr)

# 3，接听状态
listen_socket.listen(4)  # 设置套接字成监听,4表示一个己连接队列长度
print('listening...')

# 4.等待客户端来请求

# 父进程只专注接受连接请求
while True:
    # 接受连接请求，创建连接套接字，用于客户端间通信
    connect_socket, client_addr = listen_socket.accept()  # accept默认会引起阻塞
    # 新创建连接用的socket, 客户端的地址
    # print(connect_socket)
    print(client_addr)

    # 每当来新的客户端连接，创建子进程，由子进程和客户端通信
    process_do_service = Process(target=do_service, args=(connect_socket,))
    process_do_service.start()

    # 父进程，关闭connect_socket
    connect_socket.close()