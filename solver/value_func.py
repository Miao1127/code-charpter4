# _*_ coding:utf-8 _*_
# 时间：2020/4/14 21:00
# 作者：苗润龙
# 功能：1.计算单个机动动作的自信息价值；2.计算一个策略的自信息价值
# 重要提示：1. 默认参数有传感器探测半径和栅格尺寸； 2.将-60、-30、0、30、60度五个动作进行编号，分别为a1、a2、a3、a4、a5；
#           3. 艏向角变化时需要考虑越过180度的情况；
# 还存在的问题：1.行列号对应问题,具体表现为存储到csv文件中，行列号互换了，解决办法：存储前把列表转置一下；
#               2.有些栅格概率计算值为0，

import numpy as np
import copy
from solver import position2grid as pg
from usv_model import sensor_model as sm


def area_value(x, y, phi, grid_map_list, r=5, r_max=6, d=1):
    """
    计算usv探测范围内总的自信息增益
    :param x:usv在X轴坐标位置
    :param y:usv在Y轴坐标位置
    :param phi:usv艏向角，弧度值
    :param r: 传感器有效探测半径
    :param r_max:传感器的最大探测半径
    :param d:栅格尺寸
    :param grid_map_list:栅格地图，记录栅格的确定度
    :return:5个机动动作的自信息增益列表，以及更新后的概率地图
    """
    # 划分usv探测范围内的栅格
    action_grid_dict = pg.grid_of_action(x, y, phi, r, d, grid_map_list)
    # 为例依次取出字典中的栅格序列
    key_name = ['1', '2', '3', '4', '5', '6', '7', '8', '9', '10', '11', '12', '13']
    # usv探测范围内的信息增益
    area_v = 0
    rows, cols = np.array(grid_map_list).shape
    # 用于更新概率图
    for k in range(len(key_name)):
        key = str(k+1)
        for grid in action_grid_dict[key]:
            # 1.获得需要计算的栅格编号
            [mm, nn] = grid
            if mm < 0 or nn < 0 or mm >= rows or nn >= cols:
                continue
            # 排除usv所在的栅格和存在障碍物的栅格
            if grid_map_list[mm][nn] == 0:
                continue
            # 2.计算当前栅格的探测确定度
            p1 = sm.sensor(mm, nn, x, y, phi, d, r_max)
            # 3.读取当前栅格历史记录中的探测确定度
            p2 = grid_map_list[mm][nn]
            # 4.计算自信息增益
            # 排除误警
            try:
                if p2 > p1:
                    continue
                elif p1 <= 0 or p2 <= 0:   # 排除噪声干扰产生的传感器误警和概率地图中确定度为零的障碍物栅格
                    continue
                else:
                    g_value = np.log(p1) - np.log(p2)
            except Exception as err:
                print(err)
                print("栅格概率@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@")
                print(p1, p2)
            # 5.累加自信息增益
            area_v += g_value
    return area_v


def action_value(x, y, phi, grid_map_list, r=5, r_max=6, d=1):
    """
    计算usv采取各个动作的的自信息增益
    :param x:usv在X轴坐标位置
    :param y:usv在Y轴坐标位置
    :param phi:usv艏向角，弧度值
    :param r: 传感器有效探测半径
    :param r_max:传感器的最大探测半径
    :param d:栅格尺寸
    :param grid_map_list:栅格地图，记录栅格的确定度
    :return:5个机动动作的自信息增益列表，以及更新后的概率地图
    """
    # 划分usv探测范围内的栅格
    action_grid_dict = pg.grid_of_action(x, y, phi, r, d, grid_map_list)
    # 为例依次取出字典中的栅格序列
    key_name = ['1', '2', '3', '4', '5', '6', '7', '8', '9', '10', '11', '12', '13']
    # 用于存储不同范围内的栅格信息增益值
    value_list = np.zeros(len(action_grid_dict))
    # 用于更新概率图
    new_map = copy.deepcopy(grid_map_list)
    for k in range(len(key_name)):
        key = str(k+1)
        for grid in action_grid_dict[key]:
            # 1.获得需要计算的栅格编号
            [mm, nn] = grid
            # 2.计算当前栅格的探测确定度
            p1 = sm.sensor(mm, nn, x, y, phi, d, r_max)
            # 3.读取当前栅格历史记录中的探测确定度
            p2 = grid_map_list[mm][nn]
            # 4.计算自信息增益以及更新概率图
            # 排除误警
            if p2 > p1:
                continue
            elif p1 <= 0 or p2 <= 0:  # 排除噪声干扰产生的传感器误警和概率地图中确定度为零的障碍物栅格
                continue
            else:
                g_value = np.log(p1) - np.log(p2)
                new_map[mm][nn] = p1
            # 5.累加自信息增益
            value_list[k] += g_value
    # 将13个不同范围的自信息增益转换为五个机动动作的自信息增益
    a_value = np.zeros(5)
    for i in range(5):
        a_value[i] = value_list[2*i+1] + value_list[2*i+2] + value_list[2*i+3]
    return a_value, new_map


def policy_value(x, y, phi, r, d, b, gama, grid_map_list):
    """
    计算usv采用连续三个机动动作的行为策略获得的自信息增益
    :param x: usv在X轴坐标位置
    :param y: usv在Y轴坐标位置
    :param phi: usv艏向角，弧度值
    :param r: 传感器探测范围
    :param d: 栅格尺寸
    :param b: usv采用的行为策略
    :param gama: 增益折扣系数
    :param grid_map_list: 栅格概率地图
    :return: 行为策略的自信息增益
    """
    if len(b) == 0:
        return 0

    # 1.计算离开当前位置的收益
    if len(b) > 0:
        a_value, new_map = action_value(x, y, phi, grid_map_list)
        if b[0] == 'a1':
            policy_reward = a_value[0]
        elif b[0] == 'a2':
            policy_reward = a_value[1]
        elif b[0] == 'a3':
            policy_reward = a_value[2]
        elif b[0] == 'a4':
            policy_reward = a_value[3]
        elif b[0] == 'a5':
            policy_reward = a_value[4]

        # 2.将当前时刻看作k时刻，根据usv选择的机动动作b1，计算目标栅格pk+1（k+1时刻可能到达的位置）和其两侧可能到达的栅格，
        #   以及对应的艏向角
        [phi_n_1, phi, phi_p_1] = pg.state_grid(x, y, phi, r, d, b[0])
        [x, y] = [x + r * np.cos(phi), y + r * np.sin(phi)]
        [x_n, y_n] = [x + r * np.cos(phi_n_1), y + r * np.sin(phi_n_1)]
        [x_p, y_p] = [x + r * np.cos(phi_p_1), y + r * np.sin(phi_p_1)]

        # 3.根据k+1时刻可能到达目标栅格、两侧栅格和对应艏向角计算探测范围内本阶段总的未来自信息增益（三个栅格）
        state_p = 0.9 + np.random.normal(loc=0, scale=0.1, size=None)
        if state_p > 1:
            state_p = 0.99
        policy_reward += gama * (state_p*area_value(x, y, phi, grid_map_list)
                                 + 0.5*(1-state_p)*area_value(x_n, y_n, phi_n_1, grid_map_list)
                                 + 0.5*(1-state_p)*area_value(x_p, y_p, phi_p_1, grid_map_list))

    # 4.假设usv到达pk+1栅格，根据此时位置和艏向角计算usv选择b2机动动作能够到达的目标栅格pk+2（k+2时刻可能到达的位置）及
    #   其两侧可能到达的栅格
    if len(b) > 1:
        [phi_n_2, phi, phi_p_2] = pg.state_grid(x, y, phi, r, d, b[1])
        [x, y] = [x + r * np.cos(phi), y + r * np.sin(phi)]
        [x_n, y_n] = [x + r * np.cos(phi_n_2), y + r * np.sin(phi_n_2)]
        [x_p, y_p] = [x + r * np.cos(phi_p_2), y + r * np.sin(phi_p_2)]

        # 5.根据k+2时刻可能到达目标栅格、两侧栅格和对应艏向角计算探测范围内本阶段总的未来自信息增益（三个栅格）
        state_p = 0.9 + np.random.normal(loc=0, scale=0.1, size=None)
        if state_p > 1:
            state_p = 0.99
        policy_reward += gama * (state_p*area_value(x, y, phi, grid_map_list)
                                 + 0.5*(1-state_p)*area_value(x_n, y_n, phi_n_2, grid_map_list)
                                 + 0.5*(1-state_p)*area_value(x_p, y_p, phi_p_2, grid_map_list))

    # 6.假设usv到达pk+1栅格，根据此时位置和艏向角计算usv选择b2机动动作能够到达的目标栅格pk+3（k+3时刻可能到达的位置）及
    #   及其两侧可能到达的栅格
    if len(b) > 2:
        [phi_n_3, phi, phi_p_3] = pg.state_grid(x, y, phi, r, d, b[2])
        [x, y] = [x + r * np.cos(phi), y + r * np.sin(phi)]
        [x_n, y_n] = [x + r * np.cos(phi_n_3), y + r * np.sin(phi_n_3)]
        [x_p, y_p] = [x + r * np.cos(phi_p_3), y + r * np.sin(phi_p_3)]

        # 7.根据k+3时刻可能到达目标栅格、两侧栅格和对应艏向角计算探测范围内本阶段总的自信息增益（三个栅格）
        state_p = 0.9 + np.random.normal(loc=0, scale=0.1, size=None)
        if state_p > 1:
            state_p = 0.99
        policy_reward += gama * (state_p*area_value(x, y, phi, grid_map_list)
                                 + 0.5*(1-state_p)*area_value(x_n, y_n, phi_n_3, grid_map_list)
                                 + 0.5*(1-state_p)*area_value(x_p, y_p, phi_p_3, grid_map_list))

    return policy_reward


if __name__ == '__main__':
    # 1.测试单个动作的自信息增益
    data_map = np.ones((100, 100)) * 0.001
    value, n_map = action_value(5.5, 5.5, 0, data_map)
    np.savetxt('map_data.csv', n_map.T, delimiter=',')
    # print(data_map)
    # print(new_map)
    print(value)
    # 2.测试usv行为策略自信息增益
    p_value = policy_value(5.5, 5.5, 0, 5, 1, 'a1', 'a3', 'a4', 0.2, data_map)
    print(p_value)
    # 3.栅格概率地图概率均匀情况下，策略遍历测试
    action = ['a1', 'a2', 'a3', 'a4', 'a5']
    value_best = 0
    for b1 in action:
        for b2 in action:
            for b3 in action:
                p_value = policy_value(5.5, 5.5, 0, 5, 1, b1, b2, b3, 0.2, data_map)
                if p_value > value_best:
                    value_best = p_value
                    b = [b1, b2, b3]
    print(b, value_best)
    # 4.栅格上半部分概率大，下半部分概率小情况下，策略遍历测试
    value_best = 0
    for i in range(100):
        for j in range(50):
            data_map[i][j] = 0.5
    for b1 in action:
        for b2 in action:
            for b3 in action:
                p_value = policy_value(5.5, 5.5, 0, 5, 1, b1, b2, b3, 0.2, data_map)
                if p_value > value_best:
                    value_best = p_value
                    b = [b1, b2, b3]
    print(b, value_best)
