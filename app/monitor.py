# _*_ coding:utf-8 _*_
# 开发人员：103中山分队-苗润龙
# 开发时间：2020/04/20 21:05
# 文件名：monitor.py
# 开发工具：PyCharm
# 功能：串口通信版马尔科夫决策监控端
# 注意：画图栅格形式为(行编号在y轴，列编号在x轴)
# 改进：增加了usv的艏向


import time
import threading
import serial.tools.list_ports
from queue import Queue
import matplotlib.pyplot as plt
import numpy as np
from mapping import zone_plot
from file_operator import read_csv


# 启动集群协同任务
class StartSwarm:
    def __init__(self, usv_list):
        self._running = True
        self.usv_list = usv_list

    def terminate(self):
        self._running = False

    def run(self):
        while self._running:
            string_to_send = 'C' + '$' + str(self.usv_list) + '$' + '\n'
            ser.write(str.encode(string_to_send))
            print("sending message(%s)..." % string_to_send)
            time.sleep(T_send)
            print(time.time())


def send_cmd(command):
    """
    发送指令
    :return:
    """
    while True:
        string_to_send = str(command) + '$' + '\n'
        ser.write(str.encode(string_to_send))
        print("sending message(%s)..." % string_to_send)
        time.sleep(T_send)
        print(time.time())


def read_msg(r_q):
    """
    接收信息
    :return:
    """
    print('开始接收数据')
    while True:
        if ser.in_waiting:
            try:
                data_received = ser.readline().decode("utf-8")
                data = data_received.split('$')
                if len(data) == 3 and data[2] == '\n':
                    r_q.put(data)
                    print("接收到的数据为：%s" % data)
            except:
                continue


# 绘制usv位置和轨迹
def plot_vehicle(x, y, theta, x_traj, y_traj, color):  # pragma: no cover

    # Corners of triangular vehicle when pointing to the right (0 radians)
    p1_i = np.array([0.5, 0, 1]).T
    p2_i = np.array([-0.5, 0.25, 1]).T
    p3_i = np.array([-0.5, -0.25, 1]).T

    T = transformation_matrix(x, y, theta)
    p1 = np.matmul(T, p1_i)
    p2 = np.matmul(T, p2_i)
    p3 = np.matmul(T, p3_i)

    plt.plot([p1[0], p2[0]], [p1[1], p2[1]], color + '-')
    plt.plot([p2[0], p3[0]], [p2[1], p3[1]], color + '-')
    plt.plot([p3[0], p1[0]], [p3[1], p1[1]], color + '-')

    plt.plot(x_traj, y_traj, color + '--')

    plt.pause(dt)


# 转移矩阵
def transformation_matrix(x, y, theta):
    return np.array([
        [np.cos(theta), -np.sin(theta), x],
        [np.sin(theta), np.cos(theta), y],
        [0, 0, 1]
    ])


def draw_animation(d_q):
    """
    每次画面刷新即把所有的位置和轨迹重新画一遍
    :return:
    """
    print('usv_dict:%s' % usv_dict)
    usv_path = {}            # 存储各个usv航行路径
    usv_position = {}        # 记录各个usv的当前位置
    path_flag = {}          # 标记是否已创建第k个usv的路径键值
    for key in usv_dict:
        usv_position[key] = usv_dict[key][-1]
        usv_path[key] = {}                                   # 初始化usv路径字典
        path_flag[key] = 0                                  # 标记是否已创建第k段遍历路径的列表
    color_dict = {'##1': 'm', '##2': 'y', '##3': 'g', '##4': 'b', '##5': 'c'}  # 各个usv轨迹颜色设置
    # 创建一个字典，用于存储各个usv的当前位置信息和轨迹信息，格式为：usv_dict = {'#1'：[[x, y],  [x_tra, y_tra]]}
    # 当前位置的更新方式为对usv_dict['#1'][0]赋值，轨迹更新方式为对usv_dict['#1']追加元素
    fig = plt.figure()
    ax = fig.add_subplot(111)
    # 加载地图数据
    map_data = read_csv.run('../file_operator/map_data_no_obstacle.csv')
    ox, oy = zone_plot.load_map(map_data)
    while True:
        r_data = d_q.get()
        if len(r_data) == 3 and r_data[0] == 'U':
            try:
                s_msg = eval(r_data[1])                        # 接收到usv信息格式为：{"##1": [0, 0, 1, 0.25, 11, 5, 0]}
                print('s_msg:%s' % s_msg)
                usv_num = list(s_msg.keys())[0]
                if len(s_msg[usv_num]) == 7:
                    x = float(s_msg[usv_num][5])  # 列对应图中x
                    y = float(s_msg[usv_num][4])  # 行对应图中y
                    phi = float(s_msg[usv_num][6])  # 艏向角

                    # 记录usv当前位置和艏向角
                    usv_position[usv_num][0] = x
                    usv_position[usv_num][1] = y
                    usv_position[usv_num][2] = phi

                    # 将当前位置和艏向加入轨迹
                    if path_flag[usv_num] == 0:
                        usv_path[usv_num].update(np.array([[x, y, phi]]))  # 记录轨迹起点
                        path_flag[usv_num] = 0
                    else:
                        usv_path[usv_num] = np.append(usv_path[usv_num], [[x, y, phi]], axis=0)  # 追加轨迹点
                print('\n usv_dict_cover:%s' % usv_path)
                print('\n usv_position:%s' % usv_position)
            except Exception as err:
                print('err:')
                print(err)
        try:
            if show_animation:  # pragma: no cover
                plt.cla()              # 清除当前图形中的当前活动轴，其他轴不受影响
                # 绘制地图障碍物信息
                ax.invert_yaxis()      # y轴坐标刻度从上到下递增
                ax.xaxis.tick_top()    # x坐标轴置于图像上方
                plt.plot(ox, oy, ".k")
                plt.grid(True)
                plt.axis("equal")

                # 绘制各个usv位置和轨迹
                for key in usv_path:
                    # 绘制遍历轨迹
                    if len(usv_path[key]) >= 2:
                        plot_vehicle(usv_position[key][0], usv_position[key][1], usv_position[key][2],
                                     usv_path[key][1:, 0], usv_path[key][1:, 1], color_dict[key])
                        plt.text(usv_position[key][0], usv_position[key][1], key, fontsize=12)  # 添加文字
                    plt.plot(usv_dict[key][0][0], usv_dict[key][0][1], 'k*')                # 绘制各个usv起始位置
                plt.pause(dt)
        except Exception as err:
            print('err:')
            print(err)
            # TODO 判断终止程序flag，若激活，则保存图像，并终止本程序


def multi_threading():
    i_usv_list = []                   # 记录初始化过程中的usv编号
    s_usv_list = []                   # 记录集群任务开始后usv编号，后期解决集群成员变化问题，需要修改此处
    # 需要在程序运行过程中停止的线程参考下方t线程编写
    start_swarm = StartSwarm(i_usv_list)
    t = threading.Thread(target=start_swarm.run)

    # 在程序运行过程中不需要停止的线程置于下方
    t1 = threading.Thread(target=read_msg, args=(q1,))   # 监听usv发送的响应启动命令的信息
    t2 = threading.Thread(target=draw_animation, args=(q1,))

    t1.start()                        # 启动串口监听

    while True:                       # 此部分代码为了解决监控端掉线重连问题
        data_serial = q1.get()
        if len(data_serial) == 3:
            msg = data_serial[0]
            if msg[0] == 'I':
                flag = 0
                break
            elif msg[0] == 'S':
                flag = 1
                break

    while True:                       # 查看每个usv是否与监控端建立连接
        data_serial = q1.get()

        # 接收各个usv发送的初始化集群信息，并处理
        if len(data_serial) == 3 and flag == 0 and data_serial[0] == 'I':
            msg = eval(data_serial[1])  # 接收到usv信息格式为：{"##1": [0, 0, 1, 0.25, 11, 5]}
            usv_num = list(msg.keys())[0]
            if usv_num not in i_usv_list and data_serial[0] == 'I':
                i_usv_list.append(usv_num)
                if len(i_usv_list) == total_usv_num:
                    print("集群中所有usv个体已连接至监控端...")
                    t.start()                   # 启动发送开启集群协同指令
                    time.sleep(0.5)               # 等待各个usv接收开启集群任务指令
                    flag = 1

        # 接收各个usv集群任务过程中状态信息，并在图像上实时显示
        if len(data_serial) == 3 and flag == 1 and data_serial[0] == 'S':
            msg = eval(data_serial[1])  # 接收到usv信息格式为：{"##1": ["#0", 0, 1, 0.25, 11, 5]}
            usv_num = list(msg.keys())[0]
            if usv_num not in s_usv_list and data_serial[0] == 'S':
                s_usv_list.append(usv_num)
                x_usv = msg[usv_num][5]   # 列号对应x轴
                y_usv = msg[usv_num][4]   # 行号对应y轴
                usv_dict[usv_num] = np.array([[float(x_usv), float(y_usv)]])                              # 起点
                usv_dict[usv_num] = np.append(usv_dict[usv_num], [[float(x_usv), float(y_usv)]], axis=0)  # 路径
                if len(s_usv_list) == total_usv_num:
                    print("启动集群协同任务...")
                    start_swarm.terminate()  # 停止向集群发送启动指令线程
                    t2.start()               # 开启绘图线程
                    break


if __name__ == '__main__':
    # 线程间信息传递
    # 初始阶段，将串口接收到的usv响应监控端启动命令的响应信息传递给multi_threading()用于统计usv数目
    # 各个usv连接到监控端后，将串口接收到的各个usv状态信息传递给draw_animation()，用于实时显示各个usv的状态
    q1 = Queue()  # 用于接收数据
    # q2 = Queue()  # 用于发送数据，用于以后改进，监控端向各个usv发送应急指令

    # 串口通信参数
    serial_port = 'com14'
    serial_baud = 115200
    time_out = 10
    T_send = 0.5

    # 集群参数设置
    total_usv_num = 5         # 集群中usv个数
    usv_dict = {}             # 存储usv起始位置

    # 图像显示参数设置
    show_animation = True
    dt = 0.01

    # 开启串口通信端口
    ser = serial.Serial(serial_port, baudrate=serial_baud, timeout=time_out)
    time.sleep(0.5)

    # 开启线程
    multi_threading()
