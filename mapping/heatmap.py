# _*_ coding:utf-8 _*_
# 开发人员：103中山分队-苗润龙
# 开发时间：2019/12/910:52
# 文件名：heatmap.py
# 开发工具：PyCharm
# 功能：绘制任务区域栅格热力图，并将热力图以heat_map.png名字保存在当前文件夹内

import matplotlib.pyplot as plt
import matplotlib as mpl
import pandas as pd
import seaborn as sns
import numpy as np
from file_operator import read_csv


def heat_map(map_data):
    """
    绘制热力图
    :param map_data: 概率图数据（x,y,p)
    :return: 显示热力图
    """
    mpl.rcParams['font.sans-serif'] = ['SimHei']       # 将全局字体设为黑体
    mpl.rcParams['axes.unicode_minus'] = False

    df = pd.DataFrame(map_data)
    f, ax = plt.subplots(figsize=(25, 15))
    c_map = sns.diverging_palette(200, 8, as_cmap=True)  # 设置颜色sns.husl_palette(10, l=.4)
    h = sns.heatmap(df, cmap=c_map, vmax=1, vmin=0, linewidths=0.01, square=True, annot=False, cbar=False,
                    annot_kws={'size': 25, 'color': 'black'}, fmt='0.0001f', ax=ax)    # 热力图
    cb = h.figure.colorbar(h.collections[0])            # 显示colorbar
    cb.ax.tick_params(labelsize=35)                     # 设置colorbar刻度字体大小
    cb.ax.set_ylabel('概率', fontsize=35)               # 设置colorbar标题大小
    # ax.invert_yaxis()                                 # 翻转坐标轴
    ax.set_ylim(bottom=130, top=0)                      # y轴坐标刻度从上到下递增
    ax.xaxis.tick_top()                                 # x坐标轴置于图像上方
    ax.set_title('概率图', x=0.5, y=1.05, fontsize=45)   # 将坐标原点置于图像左上方
    plt.xticks(np.arange(0, df.shape[1]+1, 20), np.arange(0, df.shape[1]+1, 20), fontsize=35, rotation=0)
    plt.yticks(np.arange(0, df.shape[0], 20), np.arange(0, df.shape[0], 20), fontsize=35)
    # plt.show()
    plt.savefig('heat_map.png')                             # 输出格式eps, pdf, pgf, png, ps, raw, rgba, svg, svgz.
    print("热力图保存完成...")
    
    
if __name__ == '__main__':
    data = read_csv.run('../file_operator/matrix_map_data.csv')
    heat_map(data)
