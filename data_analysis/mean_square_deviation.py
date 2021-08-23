# _*_ coding:utf-8 _*_
# 开发人员：103中山分队-苗润龙
# 开发时间：2020/11/99:57
# 文件名：mean_square_deviation.py
# 开发工具：PyCharm
# 功能：计算有效栅格位置分布离散程度，标准差

from file_operator import read_csv, makedir
import os
import numpy as np
import pandas as pd
import math


def run(map_data):
    c = 0
    d = 0
    r = 0

    # 2.统计每个时刻搜索图中的有效栅格数目和坐标位置
    row, col = map_data.shape
    p = []
    for i in range(row):
        for j in range(col):
            if map_data[i][j] >= 0.5:
                c += 1
                d = d + i
                p.append(i)
    # 3.计算每个时刻搜索图中有效栅格坐标平均值
    p_mean = d/c
    # 4.计算标准差
    p = np.array(p)
    p = p - p_mean
    total = 0
    for kk in range(len(p)):
        total = total + p[kk]
    if c != 0:
        r = math.sqrt(total**2/c)
    return r


if __name__ == '__main__':
    map_data = np.array([[0.5, 0.1, 0.1], [0.1, 0.1, 0.1], [0.5, 0.1, 0.5]])
    result = run(map_data)
    print(result)
