# _*_ coding:utf-8 _*_
# 开发人员：103中山分队-苗润龙
# 开发时间：2020/04/22 16:00
# 文件名：usv_v1.py
# 开发工具：PyCharm
# 功能：模拟 #1 usv，马尔科夫决策局部协同搜索，具备串口收发功能和运动能力.
# 过程描述：1. 初始化栅格概率地图和随机生成USV起始位置，此时记为k时刻；2.将当前USV探测范围内的栅格概率更新到栅格概率地图
#              中，另外，当前USV需要通过集群网络，将探测到的概率信息广播给集群中其他USV；3.遍历USV的所有可选行为策略，行
#              为策略中包含 三个未来可能采用的机动动作；4.选择自信息增益最大的行为策略，将此策略下生成的三个航点和USV当
#              前位置进行曲线 拟合，形成期望航线，USV沿此航线航行，当USV即将航行出k时刻探测范围时，进行下一次马尔科夫决
#              策，重复第2-4步。


import time
import threading
import serial.tools.list_ports
import numpy as np
from queue import Queue, LifoQueue
import copy
import random
from file_operator import read_csv
from solver import value_func as vf
from solver import position2grid as pg
from usv_model import sensor_model as sm


def read_msg(rc_q, ri_q, rs_q, rp_q):
    """
    接收信息
    rc_q  用于从接收端接收监控端的指令
    rs_q 用于接收其他usv发送的状态信息，实现集群协同
    rp_q 用于接收其他usv的探测信息
    :return:
    """
    print('等待接收指令！')
    while True:
        if ser.in_waiting:
            data_received = ser.readline().decode("utf-8")
            print('接收到的信息：')
            print(str(data_received))
            print('*' * 20)
            data = data_received.split('$')
            try:
                if len(data) == 3 and data[2] == '\n':
                    if data[0] == 'C':                      # 来自监控端的指令
                        rc_q.put(eval(data[1]))
                        ri_q.queue.clear()
                    elif data[0] == 'I':                    # 来自其他usv的初始化状态信息
                        ri_q.put(eval(data[1]))
                    elif data[0] == 'S' or data[0] == 'U':  # 来自其他usv的状态信息
                        if rs_q.full():
                            rs_q.queue.clear()
                        else:
                            rs_q.put(eval(data[1]))
                    elif data[0] == 'P':                    # 来自其他usv传感器探测信息
                        if rp_q.full():
                            rp_q.queue.clear()
                        else:
                            sensor_info = eval(data[1])
                            mutex.acquire()
                            map_data[sensor_info[0]][sensor_info[1]] = sensor_info[2]
                            mutex.release()
            except Exception:
                continue


def send_msg(ss_q, sp_q):
    """
    发送信息
    ss_q 用于放置当前usv的状态信息，此信息需要发送给其他usv和监控端，实现集群协同
    sp_q 用于放置当前usv的BPSO决策结果，并发送出去
    :return:
    """
    while True:
        if not ss_q.empty():                    # 发送当前usv状态信息
            ser.write(str.encode('\n'))
            string_to_send = str(ss_q.get())
            ser.write(str.encode(string_to_send))
            print("sending state message %s...\n" % string_to_send)
            time.sleep(T_send)
            print(time.time())
        if not sp_q.empty():                    # 发送当前usv的探测信息
            ser.write(str.encode('\n'))
            string_to_send = str(sp_q.get())
            ser.write(str.encode(string_to_send))
            print("sending sensor data message %s...\n" % string_to_send)
            time.sleep(T_send)
            print(time.time())


class InitUSV:
    def __init__(self, u_info):
        self.u_info = u_info
        self._running = True

    def terminate(self):
        self._running = False

    def run(self):
        while self._running:
            string_to_send = 'I' + '$' + str(self.u_info) + '$' + '\n'  # 当前usv的初始化信息
            ser.write(str.encode(string_to_send))
            time.sleep(T_send)


# 当前usv与各个分区距离求解，需要向b_solver()传递的参数有当前usv与各个分区的距离、路径
def markov_solver(rs_q, sp_q):
    """

    :param rs_q: 接受来自move_to_pose发送的usv状态信息
    :param sp_q: 将计算的期望目标点和艏向位置发送给move_to_pose，形式为[x_goal,y_goal,phi_goal]
    :return:
    """
    while True:
        if not rs_q.empty():
            usv_state = rs_q.get()
            rho = usv_state[usv_num][1]         # 更新当前usv剩余路径长度
            x = usv_state[usv_num][4]           # 更新当前usv位置行编号
            y = usv_state[usv_num][5]           # 更新当前usv位置列编号
            phi = usv_state[usv_num][6]         # 更新当前usv位置列编号
        if rho <= 1:
            # 计算usv的期望位置和艏向
            action = ['a1', 'a2', 'a3', 'a4', 'a5']
            for b1 in action:
                for b2 in action:
                    for b3 in action:
                        p_value = vf.policy_value(x, y, phi, r, d, b1, b2, b3, gama, map_data)
                        if p_value > value_best:
                            value_best = p_value
                            b = [b1, b2, b3]
            if b[0] == 'a1':
                phi1_goal = phi1_start - np.pi / 3
            if b[0] == 'a2':
                phi1_goal = phi1_start - np.pi / 6
            if b[0] == 'a4':
                phi1_goal = phi1_start + np.pi / 6
            if b[0] == 'a5':
                phi1_goal = phi1_start + np.pi / 3
            x1_goal = x1_start + r * np.cos(phi1_goal)
            y1_goal = x1_start + r * np.cos(phi1_goal)
            sp_q.put([x1_goal, y1_goal, phi1_goal])


# 当前usv运动模拟
def move_to_pose(rp_q, ss_q, s_q, gs_q):
    """
    刷新usv所处位置和栅格概率地图
    :param rp_q: 从markov_solver()接收目标位置和艏向角，形式为[x_goal,y_goal,phi_goal]
    :param ss_q: 将当前usv_info通过集群网络发送给其他usv和监控端
    :param s_q: 将当前usv_info发送给markov_solver()，用于触发计算下次期望位置和艏向角
    :param gs_q: 将栅格概率放入队列中，传递给其他USV，形式为[列号, 行号, 概率]
    :return: 无
    """

    x = start_point[0]
    y = start_point[1]
    phi = start_point[2]
    rho = 0

    while True:
        if not rp_q.empty():
            goal = rp_q.get()
            x_diff = goal[0] - x
            y_diff = goal[1] - y
            phi_goal = goal[2]
            rho = np.sqrt(x_diff ** 2 + y_diff ** 2)  # 计算距离
        if rho > 0.5:
            # 将alpha和beta的范围限制在[-π,π]
            alpha = (np.arctan2(y_diff, x_diff)       # 计算航向偏离当前位置与终点连线间角度
                     - phi + np.pi) % (2 * np.pi) - np.pi
            beta = (phi_goal - phi - alpha + np.pi) % (2 * np.pi) - np.pi  # 计算期望航向偏离当前位置与终点连线间的角度

            v = 1                                  # 匀速
            w = Kp_alpha * alpha + Kp_beta * beta  # 当前航向与期望航向夹角越大，角速度越大

            # 运动模型
            phi = phi + w * dt
            x = x + v * np.cos(phi) * dt
            y = y + v * np.sin(phi) * dt
            time.sleep(dt)

            # 更新栅格概率地图
            # 划分usv探测范围内的栅格
            action_grid_dict = pg.grid_of_action(x, y, phi, r, d)
            # 为例依次取出字典中的栅格序列
            key_name = ['1', '2', '3', '4', '5', '6', '7', '8', '9', '10', '11', '12', '13']
            for k in range(len(key_name)):
                key = str(k + 1)
                for grid in action_grid_dict[key]:
                    # 1.获得需要计算的栅格编号
                    [mm, nn] = grid
                    # 2.计算当前栅格的探测确定度
                    p = sm.sensor(mm, nn, x, y, phi, d, r_max)
                    mutex.acquire()
                    map_data[mm][nn] = p
                    mutex.release()
                    grid_info = [mm, nn, p]
                    gs_q.put(grid_info)        # 将栅格概率放入队列中，传递给其他USV

            mutex.acquire()
            usv_info[usv_num][1] = rho         # 更新当前usv剩余路径长度
            usv_info[usv_num][4] = x           # 更新当前usv位置行编号
            usv_info[usv_num][5] = y           # 更新当前usv位置列编号
            usv_info[usv_num][6] = phi         # 更新当前usv位置列编号
            mutex.release()
            state_data = 'U' + '$' + str(usv_info) + '$' + '\n'  # 当前usv状态信息
            ss_q.put(state_data)               # 将当前USV状态信息通过集群网络发送给其他usv和监控端
            s_q.put(state_data)                # 将当前USV状态信息发送给markov_solver


# 多线程
def multi_threading():
    global usv_info, usv_list, usv_dict
    # 需要在程序运行过程中停止的线程参考下方t线程编写
    ini_usv = InitUSV(usv_info)
    t = threading.Thread(target=ini_usv.run)

    # 在程序运行过程中不需要停止的线程置于下方
    t1 = threading.Thread(target=read_msg, args=(q, q0, q2, q4))          # 读取串口数据线程
    t2 = threading.Thread(target=send_msg, args=(q1, q3))                 # 发送串口数据线程
    t3 = threading.Thread(target=markov_solver, args=(q6, q5))            # 当前usv求解器
    t4 = threading.Thread(target=move_to_pose, args=(q5, q1, q6, q3))     # 当前usv运动模型线程

    t.start()   # 开启初始化发送信息线程
    t1.start()  # 开启串口接收信息线程
    while True:  # 响应监控端发出的初始化指令
        if not q0.empty():
            # 收集集群成员信息
            i_msg = q0.get()             # 接收到数据格式为：{"##2": ["#0", 0, 1, 0.25, 11, 5, 0]}
            usv_name = list(i_msg.keys())[0]
            if usv_name not in usv_list:
                usv_list.append(usv_name)
                usv_dict.update(i_msg)   # 存储各个usv初始位置信息，用于计算各个usv到达各个分区距离
        if not q.empty():
            c_msg = q.get()                  # 接收到的数据格式为：['##2', '##1']
            if len(c_msg) == len(usv_list):  # 判断是否收到监控端开启集群指令和集群成员是否都集齐
                ini_usv.terminate()          # 停止usv向监控端发送usv编号和初始位置线程
                t2.start()                   # 开启串口发送线程，定时发送usv自身的状态信息
                for i in range(5):
                    q1.put('S' + '$' + str(usv_info) + '$' + '\n')  # 回应监控端的首条信息
                    time.sleep(0.1)
                t3.start()                   # 开启求解器线程
                t4.start()                   # 开启执行器线程
                print("连接监控端...")
                break


if __name__ == '__main__':
    # 线程间信息传递
    q = LifoQueue(10)    # 采用后进先出，用于从接收端接收监控端的指令
    q0 = Queue(10)       # 用于接收其他usv初始化状态信息
    q1 = Queue(10)       # 用于放置当前usv的状态信息，此信息需要发送给其他usv，实现集群协同
    q2 = LifoQueue(10)   # 用于接收其他usv发送的最新状态信息，实现集群协同
    q3 = Queue()         # 用于放置当前usv的探测数据，并发送出去
    q4 = Queue()         # 采用后进先出队列，用于接收其他usv的探测数据
    q5 = Queue()         # 用于将markov_solver计算所得的期望位置和艏向角发送给move_to_pose()
    q6 = Queue()         # 用于将usv的状态信息发送给Markov_solver

    # 加载集群信息
    # 1.读取栅格概率地图
    map_data = read_csv.run('../file_operator/map_data_no_obstacle.csv')
    rows, cols = np.array(map_data).shape

    # 2.初始化usv的初始位置和初始艏向角，艏向角范围为[-π,π]，以及传感器参数
    usv_num = '##1'                                  # 当前usv编号
    start_point = [11, 5, 0]                         # 当前usv出发点位置和艏向角
    cruise_speed = 1                                 # 巡航速度
    cover_speed = 0.99                               # 遍历速度
    usv_info = {usv_num: [0, 0, cruise_speed, cover_speed, start_point[0], start_point[1], start_point[2]]}    # 当前usv信息
    usv_list = [usv_num]                             # usv列表初始化，首先存储当前usv编号，后续用于存储收集到的其他usv编号
    usv_dict = copy.deepcopy(usv_info)               # usv字典初始化，首先存储当前usv信息，后续用于存储收集到的其他usv的状态信息
    r = 5                                            # 传感器有效探测范围
    r_max = 6                                        # 传感器最大探测范围
    d = 1                                            # 栅格尺寸
    Kp_alpha = 15                                    # 角速度控制参数
    Kp_beta = -3                                     # 角速度控制参数
    dt = 0.01                                        # 时间步长
    gama = 0.2                                       # 自信息增益折扣系数
    # 3.判断usv初始位置是否存在障碍物
    while map_data[x1_start][y1_start] == 0:
        print('%sUSV初始位置处为障碍物所在处！' % usv_num)
        x1_start = random.randint(0, rows)
        y1_start = random.randint(0, cols)
        phi1_start = 2 * np.pi * random() - np.pi
        print('重新随机生成的%sUSV初始位置和艏向角为[%d, %d, %d]：' % usv_num, x1_start, y1_start, phi1_start)

    # 串口通信参数
    serial_port = 'com9'
    serial_baud = 115200
    time_out = 10
    T_send = 0.5

    # 开启串口通信端口
    ser = serial.Serial(serial_port, baudrate=serial_baud, timeout=time_out)
    time.sleep(0.5)

    # 定义一个全局互斥锁，用于锁定全局变量usv_dict
    mutex = threading.Lock()

    # 开启三个线程进行收发数据和建立usv运动模型
    multi_threading()

