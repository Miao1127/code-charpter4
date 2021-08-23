# _*_ coding:utf-8 _*_
# 开发人员：103中山分队-苗润龙
# 开发时间：2020/11/610:50
# 文件名：draw_heatmap.py
# 开发工具：PyCharm
# 功能：批量绘制搜索图
from mapping import zone_plot
import os
import matplotlib.pyplot as plt
import seaborn as sns
from file_operator import read_csv, makedir
import numpy as np


def run(p_data, s_path):
    fig = plt.figure(figsize=(12, 10))
    ax = fig.add_subplot(111)

    # 绘制概率图
    sns.heatmap(p_data, annot=False, fmt='.1f', cbar=True, ax=ax)
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
    c_bar.ax.tick_params(labelsize=15)
    plt.savefig(s_path)
    # plt.pause(0.1)
    plt.close()


if __name__ == '__main__':
    # 1.批量导入文件路径
    file_path = "E:/博士论文试验数据/chapter4/zigzag1/1/1607057106/"
    save_path = "E:/博士论文试验数据/chapter4/zigzag1/heatmap_figure/" + file_path.split("/")[4] + '/'
    makedir.mkdir(save_path)
    for i, j, k in os.walk(file_path):  # 读入文件夹中文件名称
        print(k)

    # 2.绘制轨迹图并保存
    for f in tuple(k):
        path_data = read_csv.run(file_path + f)  # USV编队从左下角出发以zigzag形式实施搜索任务
        time = f.split('#')[0]
        run(path_data, save_path + time + '.png')
