# _*_ coding:utf-8 _*_
# 时间： 2020/4/10 21:56
# 作者：苗润龙
# 功能：1.计算usv马尔科夫决策的五个目标栅格位置；2.将位置转化为栅格编号；3.计算探测范围内六个扇形区域内的栅格序列
# 重要说明：1.usv的艏向角范围为（-180，180]，不包含下界-180，包含上界180；2.将usv所在栅格归类到正前方的区域中，增大选择
# 正前方动作的概率，如此可以达到节约转向耗能；3.将探测区域划分为13个区域，包括6个扇形区域和7条射线，其中，5条射线与动作
# 方向重合，受到usv回转性能限制，垂直于左右舷的射线不作为机动动作的选项。

import numpy as np
import math
from file_operator import read_csv


def markov_point(x, y, phi, r):
    """
    计算马尔可夫决策过程中五个动作能够到达的栅格点位置
    :param x: usv当前x坐标位置
    :param y: usv当前y坐标位置
    :param phi: usv艏向角，弧度值
    :param r: usv的探测半径
    :return:目标栅格列表，顺序为-pi/3、-pi/6、0、pi/6、pi/3
    """
    # -pi/3度动作的栅格坐标位置
    point_n_60 = [x + r * np.cos(phi - np.pi * 1 / 3), y + r * np.sin(phi - np.pi * 1 / 3)]
    # -pi/6度动作的栅格坐标位置
    point_n_30 = [x + r * np.cos(phi - np.pi * 1 / 6), y + r * np.sin(phi - np.pi * 1 / 6)]
    # 0度动作的栅格坐标位置
    point_0 = [x + r * np.cos(phi), y + r * np.sin(phi)]
    # pi/6度动作的栅格坐标位置
    point_30 = [x + r * np.cos(phi + np.pi * 1 / 6), y + r * np.sin(phi + np.pi * 1 / 6)]
    # pi/3度动作的栅格坐标位置
    point_60 = [x + r * np.cos(phi + np.pi * 1 / 3), y + r * np.sin(phi + np.pi * 1 / 3)]
    point = point_n_60 + point_n_30 + point_0 + point_30 + point_60
    return point


def state_grid(x, y, phi, r, d, b):
    """
    计算usv采用一个动作后所可能到达的目标栅格及其两侧邻近栅格艏向角
    :param x: usv在X轴坐标位置
    :param y: usv在Y轴坐标位置
    :param phi: usv艏向角，弧度值
    :param r: 传感器探测范围
    :param d: 栅格尺寸
    :param b: usv采用的机动动作
    :return: 返回值[艏向角减邻近栅格的艏向角，目标栅格的艏向角，艏向角增邻近栅格的艏向角]
    """
    # 目标栅格航向
    if b == 'a1':
        delta = - np.pi/3
    elif b == 'a2':
        delta = - np.pi/6
    elif b == 'a3':
        delta = 0
    elif b == 'a4':
        delta = np.pi/6
    elif b == 'a5':
        delta = np.pi/3
    # 目标位置，需要判断是否越过180和-180度
    if phi + delta > np.pi:
        phi = phi + delta - 2*np.pi
    elif phi + delta < -np.pi:
        phi = phi + delta + 2*np.pi
    else:
        phi += delta
    # 目标位置
    [x_m, y_m] = [x + r * np.cos(phi), y + r * np.sin(phi)]
    # 目标位置所处栅格编号
    [mm, nn] = position2number(x_m, y_m, d)
    # 确定目标栅格两侧邻近栅格，将usv位置和目标栅格连线看作一条射线，以usv位置为旋转点，向两侧分别扫描出邻近的两个栅格
    # 目标艏向角增方向(positive)的邻近栅格，每次最大变化量为pi/18，为保证每次变化不跨越一整个栅格，实际采用的变化量为最大量的一半
    for i in range(30):
        if phi+(i+1)*np.pi/180 > np.pi:  # 判断是否跨越180度
            phi_p = phi + (i+1)*np.pi/180 - 2*np.pi
        else:
            phi_p = phi + (i+1)*np.pi/180
        [x_p, y_p] = [x + r * np.cos(phi_p), y + r * np.sin(phi_p)]
        [m_p, n_p] = position2number(x_p, y_p, d)
        if mm != m_p or nn != n_p:
            break
    [x_p, y_p] = [(m_p-0.5) * d, (n_p-0.5) * d]
    [diff_x, diff_y] = [x_p - x, y_p - y]
    phi_p = np.arctan2(diff_y, diff_x)

    # 目标艏向角减方向(negative)的邻近栅格，变化量同上
    for i in range(30):
        if phi - (i+1)*np.pi/180 < -np.pi:  # 判断是否跨越-180度
            phi_n = phi - (i+1)*np.pi/180 + 2 * np.pi
        else:
            phi_n = phi - (i+1)*np.pi/180
        [x_n, y_n] = [x + r * np.cos(phi_n), y + r * np.sin(phi_n)]
        [m_n, n_n] = position2number(x_n, y_n, d)
        if mm != m_n or nn != n_n:
            break
    [x_n, y_n] = [(m_n-0.5) * d, (n_n-0.5) * d]
    [diff_x, diff_y] = [x_n - x, y_n - y]
    phi_n = np.arctan2(diff_y, diff_x)
    return [phi_n, phi, phi_p]


def position2number(x, y, d):
    """
    计算位置所属的栅格编号
    :param x: 位置x轴坐标位置
    :param y: 位置y轴坐标位置
    :param l: 栅格尺寸
    :return: 返回列号，行号
    """
    mm = math.ceil(x / d)
    nn = math.ceil(y / d)

    return [int(mm), int(nn)]


def grid_of_action(x, y, phi, r, d, map_data):
    """
    计算马尔可夫决策过程中五个动作能够覆盖的栅格
    :param map_data: 概率地图
    :param x: usv当前x坐标位置
    :param y: usv当前y坐标位置
    :param phi: usv艏向角，弧度值
    :param r: usv的探测半径
    :param d: 栅格尺寸
    :return:目标栅格列表
    """
    # 确定usv所在栅格，并将此栅格放置于a0动作的覆盖范围内
    [m0, n0] = position2number(x, y, d)
    # 将usv探测范围分为12个区域，六个扇面和七条射线
    grid_dict = {'1': [], '2': [], '3': [], '4': [], '5': [], '6': [], '7': [[m0, n0]], '8': [], '9': [], '10': [],
                 '11': [], '12': [], '13': []}
    # 1.计算探测范围内栅格编号的左右上下边界
    if x > r:
        m_left = position2number(x-r, y, d)[0]
    else:
        m_left = 0
    m_right = position2number(x+r, y, d)[0]
    if y > r:
        n_up = position2number(x, y-r, d)[1]
    else:
        n_up = 0
    n_down = position2number(x, y+r, d)[1]
    # 2.以usv探测距离为半径画圆，遍历圆中各个栅格，判断属于哪个动作覆盖范围
    rows, cols = np.array(map_data).shape
    for mm in range(m_left, m_right+1):
        for nn in range(n_up, n_down+1):
            # 排除超过概率图边界的栅格编号
            if mm < 0 or nn < 0 or mm >= rows or nn >= cols:
                continue
            # 排除usv所在的栅格和存在障碍物的栅格
            if (mm == m0 and nn == n0) or map_data[mm][nn] == 0:
                continue
            xmn = (mm - 0.5) * d
            ymn = (nn - 0.5) * d
            distance = np.sqrt((x - xmn)**2 + (y - ymn)**2)
            if distance <= r:
                if phi < 0 and ymn - y == 0 and xmn < x:
                    angle = -np.pi
                else:
                    angle = np.arctan2(ymn - y, xmn - x)
                # 需要判断探测区域是否覆盖-180和180度射线
                if phi < -np.pi/2 and angle > 0:  # 艏向角在第二象限，目标栅格在第三或第四象限，夹角区域包含-180度
                    diff = angle - phi - 2*np.pi
                elif phi > np.pi/2 and angle < 0:  # 艏向角在第三象限，目标栅格在第一或者第二象限（注意：坐标系为附体坐标系）
                    diff = 2*np.pi - phi + angle
                else:
                    diff = angle - phi
                # -90度射线上
                if diff == -np.pi/2:
                    if len(grid_dict['1']) == 0:
                        grid_dict['1'] = [[mm, nn]]
                    else:
                        grid_dict['1'].append([mm, nn])
                # -90到-60度范围内
                elif -np.pi/3 > diff > -np.pi/2:
                    if len(grid_dict['2']) == 0:
                        grid_dict['2'] = [[mm, nn]]
                    else:
                        grid_dict['2'].append([mm, nn])
                # -60度射线上
                elif diff == -np.pi/3:
                    if len(grid_dict['3']) == 0:
                        grid_dict['3'] = [[mm, nn]]
                    else:
                        grid_dict['3'].append([mm, nn])
                # -60到-30范围内
                elif -np.pi/6 > diff > -np.pi/3:
                    if len(grid_dict['4']) == 0:
                        grid_dict['4'] = [[mm, nn]]
                    else:
                        grid_dict['4'].append([mm, nn])
                # -30度射线上
                elif diff == -np.pi/6:
                    if len(grid_dict['5']) == 0:
                        grid_dict['5'] = [[mm, nn]]
                    else:
                        grid_dict['5'].append([mm, nn])
                # -30到0度范围内
                elif 0 > diff > -np.pi/6:
                    if len(grid_dict['6']) == 0:
                        grid_dict['6'] = [[mm, nn]]
                    else:
                        grid_dict['6'].append([mm, nn])
                # 0度射线上
                elif diff == 0:
                    if len(grid_dict['7']) == 0:
                        grid_dict['7'] = [[mm, nn]]
                    else:
                        grid_dict['7'].append([mm, nn])
                # 0到30度范围内
                elif np.pi/6 > diff > 0:
                    if len(grid_dict['8']) == 0:
                        grid_dict['8'] = [[mm, nn]]
                    else:
                        grid_dict['8'].append([mm, nn])
                # 30度射线上
                elif diff == np.pi/6:
                    if len(grid_dict['9']) == 0:
                        grid_dict['9'] = [[mm, nn]]
                    else:
                        grid_dict['9'].append([mm, nn])
                # 30到60度范围内
                elif np.pi/3 > diff > np.pi/6:
                    if len(grid_dict['10']) == 0:
                        grid_dict['10'] = [[mm, nn]]
                    else:
                        grid_dict['10'].append([mm, nn])
                # 60度射线上
                elif diff == np.pi/3:
                    if len(grid_dict['11']) == 0:
                        grid_dict['11'] = [[mm, nn]]
                    else:
                        grid_dict['11'].append([mm, nn])
                # 60到90度范围内
                elif np.pi/2 > diff > np.pi/3:
                    if len(grid_dict['12']) == 0:
                        grid_dict['12'] = [[mm, nn]]
                    else:
                        grid_dict['12'].append([mm, nn])
                # 90度射线上
                elif diff == np.pi/2:
                    if len(grid_dict['13']) == 0:
                        grid_dict['13'] = [[mm, nn]]
                    else:
                        grid_dict['13'].append([mm, nn])
    return grid_dict


if __name__ == '__main__':
    # 1.测试计算马尔可夫决策下的五个动作所要到达的目标栅格点
    p = markov_point(5.5, 5.5, 0, 5)
    print(p)
    # 2.测试usv状态转移所能到达的栅格（测试四次，分别为处在坐标轴的艏向角）
    # [phi_1, phi, phi_3] = state_grid(5.5, 5.5, 0, 5, 1, 'a3')
    # [phi_1, phi, phi_3] = state_grid(5.5, 5.5, np.pi / 2, 5, 1, 'a3')
    # [phi_1, phi, phi_3] = state_grid(5.5, 5.5, np.pi, 5, 1, 'a3')
    [phi_1, phi, phi_3] = state_grid(5.5, 5.5, -np.pi/2, 5, 1, 'a3')
    print(phi_1, phi, phi_3)
    # 3.计算位置点所属栅格编号
    [m, n] = position2number(1.5, 0.7, 1)
    print([m, n])
    # 4.测试各个动作覆盖的栅格序列
    map_data_test = read_csv.run('../file_operator/map_data_test.csv')
    test_dict = grid_of_action(5.5, 5.5, -3*np.pi/4, 5, 1, map_data_test)
    print(test_dict)
