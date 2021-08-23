# _*_ coding:utf-8 _*_
# 时间：2020/4/23 14:45
# 作者：苗润龙
# 功能：tcp客户端

import threading
from socket import *
import time


def send_message(socket_client, name):
    while True:
        time.sleep(1)
        words = "client1" + str(time.time())
        message = name + "  :   " + words
        socket_client.send(message.encode())


def receive_message(socket_client):
    while True:
        data_from_server_byte = socket_client.recv(1024)
        str = data_from_server_byte.decode()
        print(str)


if __name__ == '__main__':
    client = socket()
    ip_port = ("127.0.0.1", 8080)
    client_name = 'client1'
    client.connect(ip_port)
    thread_send = threading.Thread(target=send_message, args=(client, client_name))
    thread_send.start()
    thread_receive = threading.Thread(target=receive_message, args=(client,))
    thread_receive.start()
