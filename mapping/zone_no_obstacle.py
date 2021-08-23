# _*_ coding:utf-8 _*_
# 时间： 21:36
# 作者：苗润龙
# 功能：绘制无障碍物的搜索区域

import matplotlib.pyplot as plt
from matplotlib.pyplot import MultipleLocator
from file_operator import read_csv
from mapping import zone_plot
import seaborn as sns


if __name__ == '__main__':
    # 1.加载数据
    x = read_csv.run('../file_operator/matrix_map_data_add_start_no_obstacle.csv')
    ox, oy = zone_plot.load_map(x)
    # 2.绘制热力图
    fig1 = plt.figure()
    ax1 = plt.subplot()
    ax1.invert_yaxis()        # y轴坐标刻度从上到下递增
    ax1.xaxis.tick_top()      # x坐标轴置于图像上方
    sns.heatmap(x, annot=False, ax=ax1, square=True, vmin=0, vmax=1)
    plt.grid(True)
    # 3.绘制usv位置图
    fig2 = plt.figure()
    ax2 = plt.subplot()
    ax2.invert_yaxis()  # y轴坐标刻度从上到下递增
    ax2.xaxis.tick_top()  # x坐标轴置于图像上方
    plt.plot(ox, oy, ".k")
    plt.grid(True)
    plt.axis("equal")
    plt.show()