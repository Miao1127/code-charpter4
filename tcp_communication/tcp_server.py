# _*_ coding:utf-8 _*_
# 时间：2020/4/23 14:44
# 作者：苗润龙
# 功能：tcp服务器端

from socket import *
import threading


def talk(conn, ad):
    while True:
        try:
            words_byte = conn.recv(1024)
            words = words_byte.decode()
            print("来自客户端%s端口%s的消息: " % (ad[0], ad[1]), words)
            if not words_byte:
                break
            for c in userList:
                c.send(words.encode())
        except Exception as err:
            print(err)
            break


if __name__ == '__main__':
    print("主进程开始.")
    server = socket()
    ip_port = ("127.0.0.1", 8080)
    server.bind(ip_port)
    server.listen(5)
    userDict = {}
    userList = []
    while True:
        connection, address = server.accept()
        # conn为新的socket对象，与服务器连接后的后续操作由conn去处理
        userDict[connection] = address
        userList.append(connection)
        thread = threading.Thread(target=talk, args=(connection, address))
        thread.start()
