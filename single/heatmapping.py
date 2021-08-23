# _*_ coding:utf-8 _*_
# 开发人员：103中山分队-苗润龙
# 开发时间：2020/11/38:51
# 文件名：heatmapping.py
# 开发工具：PyCharm
# 功能：绘制热力图
import matplotlib.pyplot as plt
import seaborn as sns
from file_operator import read_csv
import numpy as np


def run(map_data):
    fig = plt.figure(figsize=(12, 10))
    ax = fig.add_subplot(111)

    # 绘制概率图
    sns.heatmap(map_data, annot=False, fmt='.1f', cbar=True, ax=ax)
    x_ticks = np.arange(0, 110, 20)
    y_ticks = np.arange(0, 110, 20)
    plt.xticks(x_ticks, labels=x_ticks)
    plt.yticks(y_ticks, labels=y_ticks)
    plt.xticks(rotation=0)
    ax.invert_yaxis()          # y轴坐标刻度从上到下递增
    plt.grid(True)
    plt.tick_params(labelsize=25)
    labels = ax.get_xticklabels() + ax.get_yticklabels()
    [label.set_fontname('Times New Roman') for label in labels]
    plt.xlabel('x(m)', fontdict={'family': 'Times New Roman', 'weight': 'normal', 'size': 25})
    plt.ylabel('y(m)', fontdict={'family': 'Times New Roman', 'weight': 'normal', 'size': 25})
    c_bar = ax.collections[0].colorbar
    # here set the labelsize by 20
    c_bar.ax.tick_params(labelsize=15)

    plt.show()


if __name__ == '__main__':
    usv_num = "#1"
    data = read_csv.run('E:/博士论文试验数据/swarm1/1/1604582710/11920#1map_data.csv')
    run(data)

