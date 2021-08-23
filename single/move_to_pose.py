"""
作用：USV到达期望位置和转到艏向
By Miao at 2019/4/23
"""

import matplotlib.pyplot as plt
import numpy as np
from random import random

# 仿真参数，速度和角速度采用线性控制，现实应用中角速度可改为PID控制
Kp_rho = 20
Kp_alpha = 60
Kp_beta = -9
dt = 0.01      # 时间步长

show_animation = True


# 移动位置
def move_to_pose(x_start, y_start, theta_start, x_goal, y_goal, theta_goal):  # 参数（起始位置角度和目标位置角度）

    x = x_start
    y = y_start
    theta = theta_start

    x_diff = x_goal - x
    y_diff = y_goal - y

    x_traj, y_traj = [], []

    rho = np.sqrt(x_diff**2 + y_diff**2)  # 计算距离
    while rho > 0.1:
        x_traj.append(x)     # 在位置列表末尾添加新对象
        y_traj.append(y)

        x_diff = x_goal - x
        y_diff = y_goal - y

        # 将alpha和beta的范围限制在[-π,π]

        rho = np.sqrt(x_diff**2 + y_diff**2)                               # 计算当前位置与期望位置间的距离
        alpha = (np.arctan2(y_diff, x_diff)                                # 计算航向偏离当前位置与终点连线间角度
                 - theta + np.pi) % (2 * np.pi) - np.pi
        beta = (theta_goal - theta - alpha + np.pi) % (2 * np.pi) - np.pi  # 计算期望航向偏离当前位置与终点连线间的角度

        v = Kp_rho * rho                                                   # 距离越远速度越大，线性控制速度
        w = Kp_alpha * alpha + Kp_beta * beta                              # 当前航向与期望航向夹角越大，角速度越大

        # 运动模型
        theta = theta + w * dt
        if theta > np.pi:
            theta = theta - 2 * np.pi
        elif theta <= -np.pi:
            theta = theta + 2 * np.pi
        x = x + v * np.cos(theta) * dt
        y = y + v * np.sin(theta) * dt
        if theta > np.pi:
            theta = theta - 2 * np.pi
        elif theta <= -np.pi:
            theta = theta + 2 * np.pi
        print(x, y, theta)

        if show_animation:  # pragma: no cover
            plt.cla()                                             # 清除当前图形中的当前活动轴，其他轴不受影响
            plt.arrow(x_start, y_start, np.cos(theta_start),      # 用箭头画出起始位置和航向，参数（起始位置，终止位置……）
                      np.sin(theta_start), color='r', width=0.1)
            plt.arrow(x_goal, y_goal, np.cos(theta_goal),         # 用箭头画出期望位置和航向
                      np.sin(theta_goal), color='g', width=0.1)
            plot_vehicle(x, y, theta, x_traj, y_traj)             # 画机器人图形


# 绘制位置
def plot_vehicle(x, y, theta, x_traj, y_traj):  # pragma: no cover

    # Corners of triangular vehicle when pointing to the right (0 radians)
    p1_i = np.array([0.5, 0, 1]).T
    p2_i = np.array([-0.5, 0.25, 1]).T
    p3_i = np.array([-0.5, -0.25, 1]).T

    T = transformation_matrix(x, y, theta)
    p1 = np.matmul(T, p1_i)
    p2 = np.matmul(T, p2_i)
    p3 = np.matmul(T, p3_i)

    plt.plot([p1[0], p2[0]], [p1[1], p2[1]], 'r-')
    plt.plot([p2[0], p3[0]], [p2[1], p3[1]], 'r-')
    plt.plot([p3[0], p1[0]], [p3[1], p1[1]], 'r-')

    plt.plot(x_traj, y_traj, 'b--')

    # plt.xlim(0, 20)    # 坐标轴显示范围
    # plt.ylim(0, 20)

    plt.pause(dt)


# 转移矩阵
def transformation_matrix(x, y, theta):
    return np.array([
        [np.cos(theta), -np.sin(theta), x],
        [np.sin(theta), np.cos(theta), y],
        [0, 0, 1]
    ])


def main():

    for i in range(3):     # 运行次数
        # 随机设置初始位置和艏向，x和y的初始位置的范围为0-20内，艏向范围为[-π,π]
        x_start = 20 * random()
        y_start = 20 * random()
        theta_start = 2 * np.pi * random() - np.pi

        # 随机设置期望位置和艏向，x和y的初始位置的范围为0-20内，艏向范围为[-π,π]
        x_goal = 40 * random()
        y_goal = 40 * random()
        theta_goal = 2 * np.pi * random() - np.pi

        # 在终端显示起始位置艏向和期望位置艏向信息
        print("Initial x: %.2f m\nInitial y: %.2f m\nInitial theta: %.2f rad\n" %
              (x_start, y_start, theta_start))
        print("Goal x: %.2f m\nGoal y: %.2f m\nGoal theta: %.2f rad\n" %
              (x_goal, y_goal, theta_goal))

        # 执行移动到期望位置命令
        move_to_pose(x_start, y_start, theta_start, x_goal, y_goal, theta_goal)


if __name__ == '__main__':
    main()
