# _*_ coding:utf-8 _*_
# 开发人员：103中山分队-苗润龙
# 开发时间：2019/12/1220:50
# 文件名：draw_path.py
# 开发工具：PyCharm
# 功能：绘制路径

import numpy as np
import matplotlib.pyplot as plt
import copy
from file_operator import read_csv
from solver import a_star
from distance import path_distance


class MAP(object):
    """
    画出地图
    """

    def __init__(self, map):
        self.map = map
        self.rows, self.cols = np.array(map).shape

    def draw_init_map(self):
        """
        画出起点终点图
        :return:
        """
        plt.imshow(self.map, cmap=plt.cm.hot, interpolation='nearest', vmin=0, vmax=10)
        # plt.colorbar()
        plt.xlim(-1, self.cols)  # 设置x轴范围
        plt.ylim(-1, self.rows)  # 设置y轴范围
        my_x_ticks = np.arange(0, self.cols, 10)
        my_y_ticks = np.arange(0, self.rows, 10)
        plt.xticks(my_x_ticks)
        plt.yticks(my_y_ticks)
        plt.grid(True)
        # plt.show()

    def draw_path_open(self, a):
        """
        画出open表中的坐标点图
        :return:
        """
        map_open = copy.deepcopy(self.map)
        for i in range(a.closed.shape[1]):
            x = a.closed[:, i]

            map_open[int(x[0]), int(x[1])] = 1

        plt.imshow(map_open, cmap=plt.cm.hot, interpolation='nearest', vmin=0, vmax=10)
        # plt.colorbar()
        plt.xlim(-1, self.cols)  # 设置x轴范围
        plt.ylim(-1, self.rows)  # 设置y轴范围
        my_x_ticks = np.arange(0, self.cols, 10)
        my_y_ticks = np.arange(0, self.rows, 10)
        plt.xticks(my_x_ticks)
        plt.yticks(my_y_ticks)
        plt.grid(True)
        # plt.show()

    def draw_path_closed(self, a):
        """
        画出closed表中的坐标点图
        :return:
        """
        print('打印closed长度：')
        print(a.closed.shape[1])
        map_closed = copy.deepcopy(self.map)
        for i in range(a.closed.shape[1]):
            x = a.closed[:, i]

            map_closed[int(x[0]), int(x[1])] = 5

        plt.imshow(map_closed, cmap=plt.cm.hot, interpolation='nearest', vmin=0, vmax=10)
        # plt.colorbar()
        plt.xlim(-1, self.cols)  # 设置x轴范围
        plt.ylim(-1, self.rows)  # 设置y轴范围
        my_x_ticks = np.arange(0, self.cols, 10)
        my_y_ticks = np.arange(0, self.rows, 10)
        plt.xticks(my_x_ticks)
        plt.yticks(my_y_ticks)
        plt.grid(True)
        # plt.show()

    def draw_direction_point(self, a):
        """
        从终点开始，根据记录的方向信息，画出搜索的路径图
        :return:
        """
        # print('打印direction长度：')
        # print(a.best_path_array.shape[1])
        map_direction = copy.deepcopy(self.map)
        for i in range(a.best_path_array.shape[1]):
            x = a.best_path_array[:, i]

            map_direction[int(x[0]), int(x[1])] = 6

        plt.imshow(map_direction, cmap=plt.cm.hot, interpolation='nearest', vmin=0, vmax=10)
        # plt.colorbar()
        plt.xlim(-1, self.cols)  # 设置x轴范围
        plt.ylim(-1, self.rows)  # 设置y轴范围
        my_x_ticks = np.arange(0, self.cols, 10)
        my_y_ticks = np.arange(0, self.rows, 10)
        plt.xticks(my_x_ticks)
        plt.yticks(my_y_ticks)
        plt.grid(True)

    def draw_three_axes(self, a):
        """
        将三张图画在一个figure中
        :return:
        """
        plt.figure()
        ax1 = plt.subplot(221)

        ax2 = plt.subplot(222)
        ax3 = plt.subplot(223)
        ax4 = plt.subplot(224)
        plt.sca(ax1)
        self.draw_init_map()
        plt.sca(ax2)
        self.draw_path_open(a)
        plt.sca(ax3)
        self.draw_path_closed(a)
        plt.sca(ax4)
        self.draw_direction_point(a)
        plt.savefig('test2.png')

        plt.show()


if __name__ == '__main__':
    # 算例
    # 读入栅格数据
    data = np.array(read_csv.run('../file_operator/matrix_map_data_add_start.csv'))
    a = a_star.AStar(data, np.array([11, 5]), np.array([49, 1]), 100000)       # 实例化一个对象
    a.run()                                                               # 调用对象中的主函数方法
    path = a.path_backtrace()
    print(path)
    p_distance = path_distance.run(path[0], path[1])
    print(p_distance)
    m = MAP(data)
    m.draw_three_axes(a)