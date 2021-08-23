# _*_ coding:utf-8 _*_
# 开发人员：103中山分队-苗润龙
# 开发时间：2019/12/914:57
# 文件名：zone_plot.py
# 开发工具：PyCharm
# 功能：绘制分区划分效果图

# _*_ coding:utf-8 _*_
# 103中山分队
# 开发人员：runlong
# 开发时间：2019/11/314:33
# 文件名：vertex_plot.py
# 开发工具：PyCharm
# 功能：绘制分区顶点

import matplotlib.pyplot as plt
import numpy as np
from file_operator import read_json, read_csv


def load_map(m_data):
    """
    读取栅格地图
    :param m_data:
    :return: 障碍物占据栅格行列号
    """
    # 统计障碍物栅格
    o_x, o_y = [], []
    rows, cols = np.array(m_data).shape
    for i in range(rows):
        for j in range(cols):
            if m_data[i][j] == 0:
                o_x.append(j)  # 列号对应x轴
                o_y.append(i)  # 行号对应y轴
    return o_x, o_y


def load_grid(g_dict):
    """
    读取栅格地图
    :param g_dict:
    :return: 分区顶点占据栅格行列号
    """
    x, y = [], []
    for kk in range(len(g_dict)):
        g_key = '#' + str(kk + 1)         # 按照字典键大小顺序统计
        rows, cols = g_dict[g_key].shape
        for ii in range(rows):
                x.append(g_dict[g_key][ii][1])  # 列号
                y.append(g_dict[g_key][ii][0])  # 行号
    return x, y


if __name__ == '__main__':
    show_animation = True
    # 1.加载障碍物地图
    map_data = read_csv.run('../file_operator/matrix_map_data_add_start.csv')
    ox, oy = load_map(map_data)
    if show_animation:  # pragma: no cover
        f, ax = plt.subplots()
        ax.invert_yaxis()        # y轴坐标刻度从上到下递增
        ax.xaxis.tick_top()      # x坐标轴置于图像上方
        plt.plot(ox, oy, ".k")
        plt.grid(True)
        plt.axis("equal")

    # 2.加载分区顶点
    vertex_data = read_json.run('../zone_operator/vertex_dict.json')  # 读取顶点数据
    vx, vy = load_grid(vertex_data)
    if show_animation:  # pragma: no cover
        for i in range(len(vx)):
            plt.plot(vx[i], vy[i], "*r")
            plt.pause(0.01)

    # 3.给每个分区涂色
    outline_data = read_json.run('../zone_operator/outline_dict.json')  # 读取轮廓数据
    if show_animation:  # pragma: no cover
        for k in range(len(outline_data)):
            key = '#' + str(k + 1)
            plt.fill(outline_data[key][:, 1], outline_data[key][:, 0])
            plt.pause(0.5)
        plt.savefig('zone.png')

    plt.show()
