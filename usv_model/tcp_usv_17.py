# _*_ coding:utf-8 _*_
# 开发人员：103中山分队-苗润龙
# 开发时间：2020/04/22 16:00
# 文件名：usv_1_ip.py
# 开发工具：PyCharm
# 版本说明：IP端口传输数据模式
# 功能：模拟 #1 usv，马尔科夫决策局部协同搜索，具备串口收发功能和运动能力.
# 过程描述：1. 初始化栅格概率地图和随机生成USV起始位置，此时记为k时刻；2.将当前USV探测范围内的栅格概率更新到栅格概率地图
#              中，另外，当前USV需要通过集群网络，将探测到的概率信息广播给集群中其他USV；3.遍历USV的所有可选行为策略，行
#              为策略中包含 三个未来可能采用的机动动作；4.选择自信息增益最大的行为策略，将此策略下生成的三个航点和USV当
#              前位置进行曲线 拟合，形成期望航线，USV沿此航线航行，当USV即将航行出k时刻探测范围时，进行下一次马尔科夫决
#              策，重复第2-4步。


import time
import threading
import numpy as np
from queue import Queue, LifoQueue
import copy
import random
import seaborn as sns
from socket import *
import matplotlib.pyplot as plt
from file_operator import read_csv
from solver import value_func as vf
from solver import position2grid as pg
from usv_model import sensor_model as sm


def read_msg(rc_q, ri_q, rs_q):
    """
    接收通信信息
    :param rc_q: 用于从接收端接收监控端的指令
    :param ri_q: 用于接受其他usv的初始化信息
    :param rs_q: 用于接收其他usv发送的状态信息，实现集群协同
    :param rp_q: 用于接收其他usv的探测信息
    :return:
    """

    print('等待接收指令！')
    global map_data
    mutex_r = threading.Lock()
    while True:
        data_received = client.recv(10240)
        data_received = data_received.decode()
        if data_received:
            print('接收到的信息：')
            print(data_received)
            # print('*' * 20)
            data = data_received.split('$')
            try:
                if len(data) == 3 and data[2] == '\n':
                    if data[0] == 'C':                      # 来自监控端的指令
                        rc_q.put(eval(data[1]))
                        ri_q.queue.clear()
                    elif data[0] == 'S' or data[0] == 'I':  # 来自其他usv的初始化状态信息
                        ri_q.put(eval(data[1]))
                    elif data[0] == 'S' or data[0] == 'U':  # 来自其他usv的状态信息
                        if rs_q.full():
                            rs_q.queue.clear()
                        else:
                            rs_q.put(eval(data[1]))
                    elif data[0] == 'P':                    # 来自其他usv传感器探测信息
                        sensor_info = eval(data[1])
                        usv_num_r = list(sensor_info.keys())[0]
                        new_data = sensor_info[usv_num_r][2]
                        old_data = map_data[sensor_info[usv_num_r][0]][sensor_info[usv_num_r][1]]
                        if usv_num_r != usv_num and new_data > old_data != 0:
                            mutex_r.acquire()
                            map_data[sensor_info[usv_num_r][0]][sensor_info[usv_num_r][1]] = sensor_info[usv_num_r][2]
                            mutex_r.release()
            except Exception as err:
                print(err)
                continue


def send_msg(ss_q):
    """
    发送信息
    ss_q 用于放置当前usv的状态信息，此信息需要发送给其他usv和监控端，实现集群协同
    sp_q 用于放置当前usv的BPSO决策结果，并发送出去
    :return:
    """
    print("开启发送信息线程")
    while True:
        if not ss_q.empty():                    # 发送当前usv状态信息
            client.send(b'\n')
            string_to_send = str(ss_q.get())
            client.send(string_to_send.encode())
            # print("发送usv状态数据：%s\n" % string_to_send)
            time.sleep(t_send)
            # print(time.time())
        # if not sp_q.empty():                    # 发送当前usv的探测信息
            # client.send(b'\n')
            # string_to_send = str(sp_q.get())
            # client.send(string_to_send.encode())
            # print("发送探测数据：%s\n" % string_to_send)
            # print(time.time())


class InitUSV:
    def __init__(self, u_info):
        self.u_info = u_info
        self._running = True

    def terminate(self):
        print('关闭发送初始化信息...')
        self._running = False

    def run(self):
        while self._running:
            string_to_send = 'I' + '$' + str(self.u_info) + '$' + '\n'  # 当前usv的初始化信息
            client.send(string_to_send.encode())
            time.sleep(t_send)


def markov_solver(sp_q):
    """

    :param sp_q: 将计算的行为策略发送给move_to_pose
    :return:
    """
    print("开启求解器线程")
    global usv_info, map_data
    while True:
        rho = usv_info[usv_num][1]               # 更新当前usv剩余路径长度
        x = usv_info[usv_num][4]                 # 更新当前usv位置行编号
        y = usv_info[usv_num][5]                 # 更新当前usv位置列编号
        phi_start = usv_info[usv_num][6]         # 更新当前usv位置艏向角

        # 处理自由航行区域
        if rho <= markov_rho_threshold and usv_info[usv_num][7] == 1:
            print("开始计算下一次行为策略...")
            value_best = 0
            # b = ['a3']
            # 计算usv的期望位置和艏向
            action = ['a1', 'a2', 'a3', 'a4', 'a5']
            # 迭代行为策略
            # 第一个动作
            for b1 in action:
                # 判断行为策略中产生的目标航点是否超出边界或存在障碍物
                if b1 == 'a1':
                    phi1_goal = phi_start - np.pi / 3
                elif b1 == 'a2':
                    phi1_goal = phi_start - np.pi / 6
                elif b1 == 'a3':
                    phi1_goal = phi_start
                elif b1 == 'a4':
                    phi1_goal = phi_start + np.pi / 6
                elif b1 == 'a5':
                    phi1_goal = phi_start + np.pi / 3

                if phi1_goal >= np.pi:
                    phi1_goal = phi1_goal - 2 * np.pi
                elif phi1_goal < -np.pi:
                    phi1_goal = 2 * np.pi + phi1_goal
                x1_goal = x + r * np.cos(phi1_goal)
                y1_goal = y + r * np.sin(phi1_goal)
                x1_grid = np.ceil(x1_goal)
                y1_grid = np.ceil(y1_goal)
                # 判断采用动作后是否超出区域，若超出区域，则遍历下一个动作，若不超出，则将此动作添加到临时行为策略中
                if x1_grid < region_grid_num or x1_grid > rows-region_grid_num or y1_grid < region_grid_num \
                        or y1_grid > cols-region_grid_num or map_data[int(x1_grid)][int(y1_grid)] == 0:
                    continue
                else:
                    bb = [b1]

                # 第二个动作
                for b2 in action:
                    # 判断行为策略中产生的目标航点是否超出边界或存在障碍物
                    if b2 == 'a1':
                        phi2_goal = phi1_goal - np.pi / 3
                    elif b2 == 'a2':
                        phi2_goal = phi1_goal - np.pi / 6
                    elif b2 == 'a3':
                        phi2_goal = phi1_goal
                    elif b2 == 'a4':
                        phi2_goal = phi1_goal + np.pi / 6
                    elif b2 == 'a5':
                        phi2_goal = phi1_goal + np.pi / 3

                    if phi2_goal >= np.pi:
                        phi2_goal = phi1_goal - 2 * np.pi
                    if phi2_goal < -np.pi:
                        phi2_goal = 2 * np.pi + phi2_goal
                    x2_goal = x1_goal + r * np.cos(phi2_goal)
                    y2_goal = y1_goal + r * np.sin(phi2_goal)
                    x2_grid = np.ceil(x2_goal)
                    y2_grid = np.ceil(y2_goal)

                    # 判断采用连续采用第二动作后是否超出区域，若超出区域，计算第一个动作的收益，记录收益最高的动作，
                    # 并遍历第二动作可以采用的下一个选项，若不超出，则将动作添加到临时行为策略中，形成包含两个连续动作的临时行为策略
                    if x2_grid < region_grid_num or x2_grid > rows-region_grid_num or y2_grid < region_grid_num \
                            or y2_grid > cols-region_grid_num or map_data[int(x2_grid)][int(y2_grid)] == 0:
                        p_value = vf.policy_value(x, y, phi_start, r, d, bb, gama, map_data)
                        if p_value > value_best:
                            value_best = p_value
                            b = bb
                        continue
                    else:
                        bb = [b1, b2]

                    # 第三个动作
                    for b3 in action:
                        # 判断行为策略中产生的目标航点是否超出边界或存在障碍物
                        if b3 == 'a1':
                            phi3_goal = phi2_goal - np.pi / 3
                        elif b3 == 'a2':
                            phi3_goal = phi2_goal - np.pi / 6
                        elif b3 == 'a3':
                            phi3_goal = phi2_goal
                        elif b3 == 'a4':
                            phi3_goal = phi2_goal + np.pi / 6
                        elif b3 == 'a5':
                            phi3_goal = phi2_goal + np.pi / 3

                        if phi3_goal >= np.pi:
                            phi3_goal = phi1_goal - 2 * np.pi
                        if phi3_goal < -np.pi:
                            phi3_goal = 2 * np.pi + phi3_goal
                        x3_goal = x2_goal + r * np.cos(phi3_goal)
                        y3_goal = y2_goal + r * np.sin(phi3_goal)
                        x3_grid = np.ceil(x3_goal)
                        y3_grid = np.ceil(y3_goal)
                        # 判断采用连续采用第三动作后是否超出区域，若超出区域，计算第一个和第二个连续动作的收益，记录收益最高的动作组合，
                        # 若不超出，则将动作添加到临时行为策略中，形成包含三个连续动作的行为策略
                        if x3_grid < region_grid_num or x3_grid > rows-region_grid_num or y3_grid < region_grid_num \
                                or y3_grid > cols-region_grid_num or map_data[int(x3_grid)][int(y3_grid)] == 0:
                            p_value = vf.policy_value(x, y, phi_start, r, d, bb, gama, map_data)
                            if p_value > value_best:
                                value_best = p_value
                                b = bb
                        else:
                            bb = [b1, b2, b3]
                            p_value = vf.policy_value(x, y, phi_start, r, d, bb, gama, map_data)
                            if p_value > value_best:
                                value_best = p_value
                                b = bb
            print("采用动作策略为：%s" % b)
            mutex.acquire()
            usv_info[usv_num][7] = 0
            mutex.release()
            sp_q.put(b)

        # 处理接近边界和遇到障碍物区域
        elif usv_info[usv_num][7] == 2:
            print("采用避障策略")
            b = ['a0']
            mutex.acquire()
            usv_info[usv_num][7] = 0
            mutex.release()
            sp_q.queue.clear()
            sp_q.put(b)


# 当前usv运动模拟
def move_to_pose(rp_q):
    """
    刷新usv所处位置和栅格概率地图
    :param rp_q: 从markov_solver()接收目行为策略
    :return: 无
    """
    print("开启usv运动线程")
    global markov_flag, usv_info, map_data
    mutex_m = threading.Lock()

    x = start_point[0]
    y = start_point[1]
    phi = start_point[2]
    rho = 0
    n = 1   # 避障转向次数

    while True:
        b = []
        # 接受markov_solver产生的行为决策
        if not rp_q.empty():
            # print('接收到markov_solver计算结果')
            b = rp_q.get()
        # 执行行为决策
        for action in b:
            # 判断是否到达边界或者前方遇到障碍物
            if usv_info[usv_num][7] == 2:
                break
            # 避障动作
            if action == 'a0':
                phi_goal = phi + n*np.pi/180
                print(n)
                print("执行避障...")
            # 以下五个动作为马尔科夫决策动作
            elif action == 'a1':
                phi_goal = phi - np.pi / 3
                n = 1
            elif action == 'a2':
                phi_goal = phi - np.pi / 6
                n = 1
            elif action == 'a3':
                phi_goal = phi
                n = 1
            elif action == 'a4':
                phi_goal = phi + np.pi / 6
                n = 1
            elif action == 'a5':
                phi_goal = phi + np.pi / 3
                n = 1

            if phi_goal >= np.pi:
                phi_goal = phi_goal - 2 * np.pi
            elif phi_goal < -np.pi:
                phi_goal = 2 * np.pi + phi_goal
            phi_goal = round(phi_goal, 3)
            x_goal = x + r * np.cos(phi_goal)
            y_goal = y + r * np.sin(phi_goal)

            x_goal = round(x_goal, 3)
            y_goal = round(y_goal, 3)

            print('下一个目标点和航向为：%s,%s,%s' % (x_goal, y_goal, phi_goal))
            x_diff = x_goal - x
            y_diff = y_goal - y
            rho = np.sqrt(x_diff ** 2 + y_diff ** 2)  # 计算距离
            # print("下一个目标点为：%s" % goal)
            while rho > waypoint_threshold:

                # 1.判断usv目标航点是否超出边界或存在障碍物区域
                x_grid = np.ceil(x_goal)
                y_grid = np.ceil(y_goal)
                if x_grid < region_grid_num or x_grid > rows - region_grid_num or y_grid < region_grid_num \
                        or y_grid > cols - region_grid_num or map_data[int(x_grid)][int(y_grid)] == 0:
                    print("计算下一次避障决策...")
                    n += 1
                    mutex.acquire()
                    usv_info[usv_num][7] = 2
                    b = []
                    mutex.release()
                    break
                else:
                    # 2.运动模型
                    alpha = (np.arctan2(y_diff, x_diff)  # 计算航向偏离当前位置与终点连线间角度
                             - phi + np.pi) % (2 * np.pi) - np.pi
                    beta = (phi_goal - phi - alpha + np.pi) % (2 * np.pi) - np.pi  # 计算期望航向偏离当前位置与终点连线间的角度

                    v = Kp_rho * rho  # 距离越远速度越大，线性控制速度
                    if v < v_min:  # 限制最低速度
                        v = v_min
                    w = Kp_alpha * alpha + Kp_beta * beta  # 当前航向与期望航向夹角越大，角速度越大
                    phi = phi + w * dt
                    if phi > np.pi:
                        phi = phi - 2 * np.pi
                    elif phi <= -np.pi:
                        phi = phi + 2 * np.pi
                    x = x + v * np.cos(phi) * dt
                    y = y + v * np.sin(phi) * dt
                    x_diff = x_goal - x
                    y_diff = y_goal - y
                    rho = np.sqrt(x_diff ** 2 + y_diff ** 2)  # 计算距离
                    # print("与下一个航点距离为：%f" % rho)
                    time.sleep(dt)

                    # 3.更新栅格概率地图
                    # 划分usv探测范围内的栅格
                    action_grid_dict = pg.grid_of_action(x, y, phi, r, d, map_data)
                    # 为例依次取出字典中的栅格序列
                    key_name = ['1', '2', '3', '4', '5', '6', '7', '8', '9', '10', '11', '12', '13']
                    for k in range(len(key_name)):
                        key = str(k + 1)
                        for grid in action_grid_dict[key]:
                            # 1.获得需要计算的栅格编号
                            [mm, nn] = grid
                            # 排除超过概率图边界的栅格编号
                            if mm < 0 or nn < 0 or mm >= rows or nn >= cols:
                                continue
                            # TODO 此处添加判断map_data[mm][nn]==1的栅格，若是，则探测动态目标，记录
                            # 2.计算当前栅格的探测确定度
                            p = sm.sensor(mm, nn, x, y, phi, d, r_max)
                            if p > map_data[mm][nn] != 0:
                                mutex_m.acquire()
                                map_data[mm][nn] = round(p, 3)
                                mutex_m.release()
                                grid_info = {usv_num: [mm, nn, round(p, 3)]}
                                sensor_data = 'P' + '$' + str(grid_info) + '$' + '\n'
                                client.send(b'\n')
                                client.send(sensor_data.encode())
                    # 4.更新usv状态
                    mutex.acquire()
                    usv_info[usv_num][1] = round(rho, 3)  # 更新当前usv剩余路径长度
                    usv_info[usv_num][4] = round(x, 3)  # 更新当前usv位置行编号
                    usv_info[usv_num][5] = round(y, 3)  # 更新当前usv位置列编号
                    usv_info[usv_num][6] = round(phi, 3)  # 更新当前usv位置艏向角
                    mutex.release()
                    state_data = 'U' + '$' + str(usv_info) + '$' + '\n'  # 当前usv状态信息
                    print("广播usv当前状态：%s" % state_data)
                    client.send(b'\n')
                    client.send(state_data.encode())

        # 5.触发下一次markov_solver计算
        if usv_info[usv_num][7] == 0:
            print("触发下一次马尔可夫决策...")
            mutex.acquire()
            usv_info[usv_num][7] = 1
            mutex.release()


# 概率地图中，栅格确定度衰减
def map_data_model():
    global usv_info, map_data
    mutex_map = threading.Lock()
    if usv_info[usv_num][7] == 1:
        print("开启概率图模型")
        while True:
            time.sleep(t_map)
            print("概率地图衰减...")
            mutex_map.acquire()
            for i in range(rows):
                for j in range(cols):
                    if map_data[i][j] != 0:
                        map_data[i][j] = reduction_coefficient*map_data[i][j] \
                                         + random.uniform(-1*reduction_scale, reduction_scale)
                        # TODO 此数添加map_data[i][j]=1的栅格点作为设置的目标点，其中i和j随机选取
                        if map_data[i][j] < 0.1 and map_data[i][j] != 0:
                            map_data[i][j] = 0.1
            mutex_map.release()
            print(map_data)


# 绘制概率图
def draw_animation():
    print("开启绘制概率图线程")
    global map_data
    fig = plt.figure("集群"+usv_num+"概率图")
    ax = fig.add_subplot(111)
    while True:
        try:
            if show_animation:  # pragma: no cover
                plt.cla()              # 清除当前图形中的当前活动轴，其他轴不受影响
                # 绘制概率图
                sns.heatmap(map_data, annot=False, fmt='.1f', cbar=False, ax=ax)
                # ax.invert_yaxis()      # y轴坐标刻度从上到下递增
                ax.xaxis.tick_top()      # x坐标轴置于图像上方
                plt.grid(True)
                plt.axis("equal")
                plt.pause(dt)

        except Exception as err:
            print('err:')
            print(err)


# 多线程
def multi_threading():
    global usv_info, usv_list, usv_dict
    # 需要在程序运行过程中停止的线程参考下方t线程编写
    ini_usv = InitUSV(usv_info)
    t = threading.Thread(target=ini_usv.run)

    # 在程序运行过程中不需要停止的线程置于下方
    t1 = threading.Thread(target=read_msg, args=(q, q0, q1))              # 读取串口数据线程
    t2 = threading.Thread(target=send_msg, args=(q3,))                 # 发送串口数据线程
    t3 = threading.Thread(target=markov_solver, args=(q2,))               # 当前usv求解器
    t4 = threading.Thread(target=move_to_pose, args=(q2,))                # 当前usv运动模型线程
    t5 = threading.Thread(target=draw_animation)                          # 绘制动态概率图
    t6 = threading.Thread(target=map_data_model)                          # 概率衰减模型

    t.start()   # 开启初始化发送信息线程
    t1.start()  # 开启串口接收信息线程
    while True:  # 响应监控端发出的初始化指令
        if not q0.empty():
            # 收集集群成员信息
            i_msg = q0.get()             # 接收到数据格式为：{"##2": [0, 1, 0.25, 11, 5, 0]}
            usv_name = list(i_msg.keys())[0]
            if usv_name not in usv_list:
                usv_list.append(usv_name)
                usv_dict.update(i_msg)   # 存储各个usv初始位置信息，用于计算各个usv到达各个分区距离
        if not q.empty():
            c_msg = q.get()                  # 接收到的数据格式为：['##2', '##1']
            if len(c_msg) == len(usv_list):  # 判断是否收到监控端开启集群指令和集群成员是否都集齐
                ini_usv.terminate()          # 停止usv向监控端发送usv编号和初始位置线程
                t2.start()
                print("连接到监控端")
                for i in range(5):
                    q3.put('S' + '$' + str(usv_info) + '$' + '\n')  # 回应监控端的首条信息
                    time.sleep(0.1)
                t3.start()                   # 开启求解器线程
                t4.start()                   # 开启执行器线程
                t5.start()                   # 开启绘制概率图线程
                t6.start()                   # 开启概率图衰减模型
                break


if __name__ == '__main__':
    # 线程间信息传递
    q = LifoQueue()      # 采用后进先出，用于从接收端接收监控端的指令
    q0 = Queue()         # 用于接收其他usv初始化状态信息
    q1 = LifoQueue()     # 用于接收其他usv发送的最新状态信息，实现集群协同
    q2 = Queue()         # 用于将markov_solver计算所得的期望位置和艏向角发送给move_to_pose()
    q3 = Queue()

    show_animation = True

    # 加载集群信息
    # 1.读取栅格概率地图
    map_data = read_csv.run('../file_operator/map_data_100_test.csv')
    rows, cols = np.array(map_data).shape
    print(rows, cols)

    # 2.初始化usv的初始位置和初始艏向角，艏向角范围为[-π,π]，以及传感器参数
    usv_num = '#17'                                   # 当前usv编号
    start_point = [3, 19, np.pi*random.uniform(0, 0.5)]                          # 当前usv出发点位置和艏向角
    cruise_speed = 1                                 # 巡航速度
    cover_speed = 0.99                               # 遍历速度
    markov_flag = 0                                  # 触发下一次markov_solver计算
    usv_info = {usv_num: [0, 0, cruise_speed, cover_speed, start_point[0], start_point[1], start_point[2], markov_flag]}
    usv_list = [usv_num]                             # usv列表初始化，首先存储当前usv编号，后续用于存储收集到的其他usv编号
    usv_dict = copy.deepcopy(usv_info)               # usv字典初始化，首先存储当前usv信息，后续用于存储收集到的其他usv的状态信息
    r = 5                                            # 传感器有效探测范围
    r_max = 6                                        # 传感器最大探测范围
    d = 1                                            # 栅格尺寸
    Kp_alpha = 60                                    # 角速度控制参数
    Kp_beta = -9                                     # 角速度控制参数
    Kp_rho = 20
    dt = 0.01                                       # 运动时间步长
    v_min = 1                                        # usv最低速度
    sensor_date_t = 0.001                            # 传感器数据传输时间间隔
    gama = 0.2                                       # 自信息增益折扣系数
    region_grid_num = 2                              # 避碰栅格数量
    markov_rho_threshold = 4                         # 触发markov_solver的航点距离阈值
    waypoint_threshold = 1                           # 航点到达阈值
    t_map = 60                                       # 概率图概率变化时间间隔
    reduction_coefficient = 0.99                     # 栅格概率衰减系数
    reduction_scale = 0.001                          # 栅格概率衰减噪声标准差
    # 3.判断usv初始位置是否存在障碍物
    while map_data[start_point[0]][start_point[1]] == 0:
        print('%sUSV初始位置处为障碍物所在处！' % usv_num)
        start_point[0] = random.randint(0, rows-1)
        start_point[1] = random.randint(0, cols-1)
        phi1_start = 2 * np.pi * random.random() - np.pi
        print('重新随机生成的%sUSV初始位置和艏向角为[%d, %d, %d]：' % (usv_num, start_point[0], start_point[1], phi1_start))

    # tcp通信
    client = socket()
    ip_port = ("127.0.0.1", 8080)
    client.connect(ip_port)
    t_send = 0.5      # 发送信息时间间隔

    # 定义一个全局互斥锁，用于锁定全局变量
    mutex = threading.Lock()

    # 开启三个线程进行收发数据和建立usv运动模型
    multi_threading()

