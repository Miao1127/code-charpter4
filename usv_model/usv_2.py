# _*_ coding:utf-8 _*_
# 开发人员：103中山分队-苗润龙
# 开发时间：2019/12/418:26
# 文件名：usv_2.py
# 开发工具：PyCharm
# 功能：模拟#2 usv，具备串口收发能力和运动能力
# 注意：计算栅格形式为(行编号，列编号)

# 改进记录：2019.12.17 每类需要向外发送的信息，单独建立一个Quene信息流，有：usv状态信息（初始状态信息和过程状态信息）、当前usv与
#                     各个分区的距离、BPSO决策结果；接收到的信息，有：监控端发送的指令、其他usv状态信息（初始状态信息和过程状态信息）、
#                     其他usv与各个分区的距离、其他usv的BPSO决策结果，先在read_msg()函数中分类预处理，将其放入不同的quene中。

import time
import threading
import serial.tools.list_ports
import numpy as np
from queue import Queue, LifoQueue
import copy
import random
from file_operator import read_json, read_csv, nested_dict2json, read_nested_dict_json
from zone_operator import zone_cover_cost, zone_value, zone_cover
from solver import bpso, a_star
from distance import point2zone


def read_msg(rc_q, ri_q, rs_q, rd_q, rp_q):
    """
    接收信息
    rc_q  用于从接收端接收监控端的指令
    rs_q 用于接收其他usv发送的状态信息，实现集群协同
    rd_q 用于接收其他usv与各个分区的距离
    rp_q 用于接收其他usv的BPSO决策结果
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
                    elif data[0] == 'S' or data[0] == 'U':                    # 来自其他usv的状态信息
                        if rs_q.full():
                            rs_q.queue.clear()
                        else:
                            rs_q.put(eval(data[1]))
                    elif data[0] == 'D':                    # 其他usv与各个分区距离信息
                        if rd_q.full():
                            rd_q.queue.clear()
                        else:
                            rd_q.put(eval(data[1]))
                    elif data[0] == 'P':                    # 来自其他usv的BPSO决策信息
                        if rp_q.full():
                            rp_q.queue.clear()
                        else:
                            rp_q.put(eval(data[1]))
            except Exception:
                continue


def send_msg(ss_q, sd_q, sp_q):
    """
    发送信息
    ss_q 用于放置当前usv的状态信息，此信息需要发送给其他usv，实现集群协同
    sd_q 用于放置当前usv与各个分区的距离，并发送出去
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
        if not sd_q.empty():                    # 发送当前usv与各个分区距离信息
            ser.write(str.encode('\n'))
            string_to_send = str(sd_q.get())
            ser.write(str.encode(string_to_send))
            print("sending distance to zones message %s...\n" % string_to_send)
            time.sleep(T_send)
            print(time.time())
        if not sp_q.empty():                    # 发送当前usv的BPSO决策信息
            ser.write(str.encode('\n'))
            string_to_send = str(sp_q.get())
            ser.write(str.encode(string_to_send))
            print("sending BPSO message %s...\n" % string_to_send)
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
def solver(rs_q, sd_q, rd_q, sp_q, rp_q, p_q):
    # 参数说明：需要迭代更新的参数有usv_dict, path, usv_start_point, occupied_zone
    global map_data, zone_data, vertex_dict, usv_info, usv_list, usv_dict
    global usv_num, start_point, cruise_speed, cover_speed
    path = np.array([[start_point[0], start_point[1], cruise_speed]])  # 用于存储当前usv的航行的路径，初始化存储出发点
    usv_start_info = copy.deepcopy(usv_info)  # 巡航路径规划起始点信息
    v_list = zone_value.run(zone_data)                                         # 分区价值列表
    c_list = zone_cover_cost.run(zone_data)                                    # 分区遍历代价列表，即每个分区内的栅格数目列表
    occupied_zone = []  # 存储已经被遍历过的分区，每次进行完BPSO分区协同任务分配后，将取得一致的最终决策分区代号写入此列表
    k = 1   # 计数
    while True:
        # 初始化清空
        if not sd_q.empty():
            sd_q.queue.clear()
        if not rd_q.empty():
            rd_q.queue.clear()
        if not sp_q.empty():
            sp_q.queue.clear()
        if not rp_q.empty():
            rp_q.queue.clear()
        # 1.各个usv根据自身所在位置计算距离每个分区的最短距离，并生成到达各个分区的巡航路径选项
        print('0' * 20)
        print('第%s次分区分配' % k)
        print('0'*20)
        print('\n'*5)
        print('计算当前usv与各个分区最短距离...')
        print('输入的信息：')
        print('usv_start_info:%s' % usv_start_info)
        print('*'*10)
        print('occupied_zone: %s' % occupied_zone)
        print('*'*10)
        u2z_mat = np.zeros((len(usv_list) + 1, len(zone_data) + 1))  # 初始化usv到分区的距离矩阵

        # 1. 第一次计算距离和路径代码，运行完后，此部分代码可以注释，切换到2.
        # u2z_d, usv_path = point2zone.run(usv_start_info, occupied_zone, vertex_dict, map_data)  # 计算usv从当前位置到各个分区的距离
        # if k == 1:
        #     np.savetxt('u2z_usv2.csv', u2z_d, delimiter=',')
        #     nested_dict2json.run(usv_path, 'usv2_path')

        # 2. 从1中得到距离和路径数据后，运行此部分代码，减少仿真时间
        if k == 1:
            # ********************测试代码逻辑用*********************************
            u2z_d = read_csv.run('u2z_usv2.csv')
            usv_path = read_nested_dict_json.run('usv2_path.json')
            # ******************************************************************
        else:
            u2z_d, usv_path = point2zone.run(usv_start_info, occupied_zone, vertex_dict, map_data)  # 计算usv从当前位置到各个分区的距离

        print('输出量：')
        print('u2z_d: %s' % u2z_d)
        print('usv_path: %s' % usv_path)
        # 2.向其他usv发送计算出的距离，直至集齐其他usv发送的距离信息为止，u2z_d的形式为[[1, 22.22234, 23.4555, 354, 33.324]]
        u2z_d[0] = [np.round(x, 1) for x in u2z_d[0]]
        u2z_d = np.array(u2z_d)
        u2z_mat[int(u2z_d[0, 0])] = u2z_d                          # 将当前usv与各个分区距离添加到距离矩阵中去
        u2z_sd = list(u2z_d[0])                                    # 将ndarray转换为list，保证完整传输
        sd_dict = {'d': u2z_sd, 'k': k}
        d_send_msg = 'D' + '$' + str(sd_dict) + '$' + '\n'          # 当前usv与各个分区的距离信息

        while True:  # 当前usv发送的距离信息得到其他所有usv的回应后才停止发送
            print('向其他usv发送当前usv与各个分区最短距离...')
            sd_q.put(d_send_msg)
            if not rd_q.empty():
                rd_dict = rd_q.get()
                if len(rd_dict) == 2 and rd_dict['k'] == k:
                    d_msg = np.array(rd_dict['d'])
                    print('d_msg:%s' % d_msg)
                    if len(d_msg) == len(zone_data) + 1:
                        usv_row = int(d_msg[0])
                        u2z_mat[usv_row] = d_msg
            print('^'*20)
            print(u2z_mat)
            print('^' * 20)
            if (u2z_mat[1:, 0] != 0).all():  # 集齐其他usv距离信息后，为保证让其他usv收集到当前usv的距离信息，需要额外发送若干次
                for i in range(30):          # 额外重复发送次数需要为队列的2倍以上，避免其他usv接收队列接收满后清空
                    sd_q.put(d_send_msg)
                break
        time.sleep(5)

        # 3.BPSO分区协同分配，并将结果发送出去
        # 发送自身决策结果的同时也需要接受其他usv的决策结果
        # usv完成分区遍历任务，一旦收集完成开始决策协商。
        # 参数列表：各个usv所在分区(通过串口通信收集)，已分配分区列表(每个usv建立一个列表进行存储)，分区价值列表(zone_value.py)，
        # 分区剩余遍历路径长度，距离矩阵(加载../distance/zone2zone_distance_mat.csv)
        # 默认参数(若修改，需要进入/solver/bpso.py中设定）：种群数目，最大迭代次数，价值权重，分布权重，到达指定分区代价权重，
        # 遍历指定分区代价权重，协同时间差异代价权重
        # 3.1 更新各个usv的状态信息，一旦发现有剩余路径小于最小分区栅格数两倍的剩余路径出现，立即开启下一次的BPSO分区分配
        while True:
            if not rs_q.empty():
                u_info = rs_q.get()
                usv_dict.update(u_info)
            usv_data = list(usv_dict.values())
            usv_data = np.array(usv_data)
            path_length = [int(x) for x in usv_data[:, 1]]
            if (np.array(path_length) < min(c_list)).any():       # 判断所有的usv中是否有剩余路径长度达到下次BPSO启动阈值
                break

        # 3.2 BPSO分区分配
        print('开始BPSO...')
        print('输入量：')
        print('usv_dict:%s' % usv_dict)
        print('occupied_zone:%s' % occupied_zone)
        zone_pso = bpso.PSO(usv_dict, occupied_zone, v_list, c_list, z2z_mat, u2z_mat)
        zone_pso.init_population()
        b_result = zone_pso.iterator()  # BPSO决策结果，例如：{'##3': [10, 17], '##4': [3, 18], 'f': 0.7377635825268328}
        usv_plan = b_result
        print('输出量：')
        print('usv_plan:%s' % usv_plan)
        usv_plan['u'] = [usv_num]
        usv_plan['k'] = k
        usv_plan['r'] = random.randint(0, 10000)  # 生成随机数，防止效能指数相同时出现决策选择差异
        del u2z_mat  # 清除当前usv在当前位置与各个分区的距离矩阵
        rp_q.queue.clear()   # 清空接收决策队列

        # 4.决策方案协商，发送自身决策结果，收集其他usv决策结果并进行对比，找到f值最大者，将当前usv编号添加到响应的最后方案中
        # 决策发送协议：
        # {'##4': [3, 4], '##1': [1, 5], '##5': [15, 7], '##3': [10, 14], '##2': [3, 18],
        # 'f': 0.973871766408908, 'u':['##1']}
        # 协议说明： 键'#4'表示usv编号，值[3, 4]表示决策结果，#4 usv从所在的3分区到4分区，f表示整个决策的效能指数
        # 'u'对应的值表示usv认可的最优决策，即若此方案最优，当前usv将会把自身编号追加到此列表中，并发送出去
        while True:
            s_plan = 'P' + '$' + str(usv_plan) + '$' + '\n'  # 当前usv决策信息
            if sp_q.full():
                time.sleep(5)
            sp_q.put(s_plan)
            if not rp_q.empty():
                r_plan = rp_q.get()  # 用于存储其他usv发送的决策方案
                if r_plan['k'] == k:
                    other_plan = r_plan
                    print('other_plan:%s' % other_plan)
                    if len(other_plan['u']) != len(usv_list) or len(usv_plan['u']) != len(usv_list):
                        if other_plan['f'] > usv_plan['f'] and usv_num not in other_plan['u']:    # 接收到的其他usv的决策方案优于当前usv的决策方案
                            other_plan['u'].append(usv_num)
                            usv_plan = other_plan
                        elif other_plan['f'] > usv_plan['f'] and usv_num in other_plan['u']:    # 接收到的其他usv的决策方案优于当前usv的决策方案
                            usv_plan = other_plan
                        elif other_plan['f'] == usv_plan['f'] and usv_num not in other_plan['u']:  # 接收到其他usv的方案等效当前usv方案
                            other_plan['u'].append(usv_num)
                            usv_plan = other_plan
                        elif other_plan['f'] == usv_plan['f'] and usv_num in other_plan['u']:      # 当前usv方案最优
                            usv_plan = other_plan
                    elif len(other_plan['u']) == len(usv_list) and len(usv_plan['u']) == len(usv_list):
                        if other_plan['f'] == usv_plan['f'] and other_plan['r'] >= usv_plan['r']:  # 排除相同效能指数下的不同决策差异
                            usv_plan = other_plan
                        for i in range(20):
                            sp_q.put(s_plan)
                        break
        print('usv_plan:%s' % usv_plan)
        print('other_plan:%s' % other_plan)
        time.sleep(5)

        for key in usv_plan:  # 更新占据分区列表
            if key != 'u' and key != 'f' and key != 'k' and key != 'r':
                o_zone = int(usv_plan[key][1])
                occupied_zone = np.append(occupied_zone, [o_zone], axis=0)  # 将达成一致的分配方案中分区的编号加入到已被分配的分区列表中
                usv_dict[key][0] = usv_plan[key][1]                         # 更新各个usv进行下次BPSO分区分配时所处分区编号

        # 5.执行决策结果，若usv在初始出发位置，则需要从出发位置到决策指定分区进行路径规划，若usv在分区内，则需要确定分区遍历路径终点，以
        #   此终点为到达下一个分区路径的起点做路径规划，规划完成后usv奔向各自的指定分区，同时执行以指定分区遍历终点为各自usv起点的，到达
        #   剩余未分配分区的距离计算
        # 5.1 解析最终决策方案，将目标分区更新到usv_info， 并将目标分区和下次巡航路径规划的起点位置存储到usv_start_info中
        #     然后从第1步的巡航路径选项中选择对应巡航路径，传递给执行器
        target_zone_num = usv_plan[usv_num][1]
        mutex.acquire()
        usv_info[usv_num][0] = target_zone_num
        mutex.release()
        print('occupied_zone:%s' % occupied_zone)
        zone_key = '#' + str(target_zone_num)
        print('zone_key:%s' % zone_key)
        print('usv_path:%s' % usv_path)
        plan_path = usv_path[usv_num][zone_key]   # 路径为栅格行列号的列表[[5, 159],[4,159]]
        print('plan_path:%s' % plan_path)
        speed = np.ones(len(plan_path)) * cruise_speed  # 在路径的矩阵中追加一列巡航速度项
        plan_path = np.c_[plan_path, speed]
        print('plan_path:%s' % plan_path)
        if (plan_path[0][0:2] == path[-1][0:2]).all():  # 将巡航路径拼接到总路径上
            path = np.append(path, plan_path, axis=0)
        print('path:%s' % path)

        # 5.2 对本次分配的分区进行遍历路径规划，传递给执行器，记录遍历路径终点
        cover_start = plan_path[-1]
        print('cover_start:%s' % cover_start)
        cover_grid_list = zone_data[zone_key]
        print('cover_grid_list:%s' % cover_grid_list)
        cover_path = zone_cover.run(cover_start, cover_grid_list)
        speed = np.ones(len(cover_path)) * cover_speed
        cover_path = np.c_[cover_path, speed]
        print('cover_path:%s' % cover_path)
        if (cover_path[0][0:2] == path[-1][0:2]).all():  # 将遍历路径拼接到总路径上
            path = np.append(path, cover_path, axis=0)
            usv_start_info[usv_num][0] = target_zone_num  # 将目标分区的编号更新到usv_start_info中
            usv_start_info[usv_num][4] = path[-1][0]  # 更新下次巡航遍历的起点行编号
            usv_start_info[usv_num][5] = path[-1][1]  # 更新下次巡航遍历的起点列编号
        # 5.3 将路径传递给actor()
        p_q.put(path)
        # 5.4 将本次分配的分区编号添加到占据分区内，并判断是否还存在未分配的分区，若无，则以遍历路径终点为起点，出发点为终点，规划返航路径
        if len(occupied_zone) == len(v_list):  # 不存在还未分配的分区
            print('返航路径生成...')
            print('输入量：')
            print('path[-1]:%s' % path[-1])
            print('start_point:%s' % start_point)
            recovery_start = path[-1][0:2]
            recovery_goal = start_point  # 回到usv布放起点
            recovery = a_star.AStar(np.array(map_data), recovery_start, recovery_goal, 100000)
            recovery.run()
            recovery_path = recovery.path_backtrace()
            print('输出量：')
            print('recovery_path:%s' % recovery_path)
            recovery_speed = np.ones(len(recovery_path)) * cruise_speed
            recovery_path = np.c_[recovery_path, recovery_speed]
            path = np.append(path, recovery_path, axis=0)
            p_q.put(path)
            time.sleep(5)
            break
        # 5.5 清空path和plan_path，将本次遍历路径的终点加入到path中作为下次巡航路径的起点
        del path
        del plan_path
        del cover_grid_list
        path = np.array([[usv_start_info[usv_num][4], usv_start_info[usv_num][5], cruise_speed]])
        # 5.6 清空决策
        usv_plan.clear()
        other_plan.clear()
        # 5.7 清空队列
        time.sleep(10)
        if not sd_q.empty():
            sd_q.queue.clear()
        if not rd_q.empty():
            rd_q.queue.clear()
        if not sp_q.empty():
            sp_q.queue.clear()
        if not rp_q.empty():
            rp_q.queue.clear()
        time.sleep(0.5)
        k += 1
        print('清空决策后...')
        print('usv_plan:%s' % usv_plan)
        print('other_plan:%s' % other_plan)
        # TODO 判断是否所有的分区都被分配，若是，则向actor发送分配完毕信号，并终止本线程


# 当前usv运动模拟
def actor(ss_q, p_q):
    """
    刷新usv所处栅格位置
    ss_q 将当前usv的状态信息发送给其他usv
    sl_q 将剩余路径长度发送给其他usv
    p_q  从solver()接收路径
    l_q  将剩余路径长度发送给solver()，用于触发BPSO
    :return:
    """
    total_path = p_q.get()
    while True:
        if not p_q.empty():
            add_path = p_q.get()
            total_path = np.append(total_path, add_path, axis=0)
        if len(total_path) > 2:
            total_path = total_path[1:]
            x = total_path[0][0]  # 列编号
            y = total_path[0][1]  # 行编号
            v = total_path[0][2]  # 速度，1为巡航速度，小于1大于0为遍历速度，0为完成集群任务
            dt = 1 / v
            time.sleep(dt)
            mutex.acquire()
            usv_info[usv_num][1] = len(total_path)               # 更新当前usv剩余路径长度
            usv_info[usv_num][3] = v                             # 栅格速度项，若v=1，栅格为巡航路径栅格，若0<v<1，为遍历栅格
            usv_info[usv_num][4] = x                             # 更新当前usv位置行编号
            usv_info[usv_num][5] = y                             # 更新当前usv位置列编号
            mutex.release()
            state_data = 'U' + '$' + str(usv_info) + '$' + '\n'  # 当前usv状态信息
            ss_q.put(state_data)
        # TODO 从solvor接收一个所有分区都已被分配的信号，通过判断剩余路径长度和此信号来终止程序，终止前需要向监控端发送终止信号
        # 终止信号可以设定为0


# 多线程
def multi_threading():
    global usv_info, usv_list, usv_dict
    # 需要在程序运行过程中停止的线程参考下方t线程编写
    ini_usv = InitUSV(usv_info)
    t = threading.Thread(target=ini_usv.run)

    # 在程序运行过程中不需要停止的线程置于下方
    t1 = threading.Thread(target=read_msg, args=(q, q0, q2, q4, q6))       # 读取串口数据线程
    t2 = threading.Thread(target=send_msg, args=(q1, q3, q5))              # 发送串口数据线程
    t3 = threading.Thread(target=solver, args=(q2, q3, q4, q5, q6, q7))    # 当前usv求解器
    t4 = threading.Thread(target=actor, args=(q1, q7))                     # 当前usv运动模型线程

    t.start()   # 开启初始化发送信息线程
    t1.start()  # 开启串口接收信息线程
    while True:  # 响应监控端发出的初始化指令
        if not q0.empty():
            # 收集集群成员信息
            i_msg = q0.get()             # 接收到数据格式为：{"##2": ["#0", 0, 1, 0.25, 11, 5]}
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
    q3 = Queue(20)       # 用于放置当前usv与各个分区的距离，并发送出去
    q4 = LifoQueue(20)   # 用于接收其他usv与各个分区的距离
    q5 = Queue(10)       # 用于放置当前usv的BPSO决策结果，并发送出去
    q6 = Queue(20)       # 采用后进先出队列，用于接收其他usv的BPSO决策结果
    q7 = Queue()         # 用于将计算所得的路径发送给actor()

    # 加载集群信息
    # BPSO算法参数列表：usv所在分区，已分配分区列表，价值列表，分区剩余遍历路径长度，距离矩阵，种群数目，最大迭代次数，价值权重，
    # 分布权重，到达指定分区代价权重，遍历指定分区代价权重，协同时间差异代价权重
    z2z_mat = read_csv.run("../distance/z2z_distance_v2.csv")                  # 分区形心距离矩阵，用于在BPSO中衡量分区分散程度
    map_data = read_csv.run('../file_operator/matrix_map_data_add_start.csv')  # 读取地图信息
    zone_data = read_json.run("../zone_operator/split_dict.json")              # 读取分区信息
    vertex_dict = read_json.run('../zone_operator/vertex_dict.json')           # 读取分区顶点信息

    # 在计算分区价值时，需要排除已被遍历过的分区
    usv_num = '##2'                                  # 当前usv编号
    start_point = [13, 9]                            # 当前usv出发点
    cruise_speed = 1                                 # 巡航速度
    cover_speed = 0.99                               # 遍历速度
    usv_info = {usv_num: [0, 0, cruise_speed, cover_speed, start_point[0], start_point[1]]}    # 当前usv信息
    usv_list = [usv_num]                             # usv列表初始化，首先存储当前usv编号，后续用于存储收集到的其他usv编号
    usv_dict = copy.deepcopy(usv_info)               # usv字典初始化，首先存储当前usv信息，后续用于存储收集到的其他usv的状态信息

    # 串口通信参数
    serial_port = 'com10'
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



