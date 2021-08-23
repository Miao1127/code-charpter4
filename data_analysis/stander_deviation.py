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


def run(p_value, f_path):
    # 1.读取文件夹中各个文件的文件名，即不同时刻的搜索图
    for i, j, k in os.walk(f_path):
        print(k)
    t = np.zeros(len(k))
    c = np.zeros(len(k))
    d = np.zeros(len(k))
    r = np.zeros(len(k))

    # 2.统计每个时刻搜索图中的有效栅格数目和坐标位置
    for f in range(len(k)):
        t[f] = int(k[f].split("#")[0])
        map_data = np.array(read_csv.run(f_path + k[f]))
        row, col = map_data.shape
        p = []
        for i in range(row):
            for j in range(col):
                if map_data[i][j] >= p_value:
                    c[f] += 1
                    d[f] = d[f] + j
                    p.append(j)
        # 3.计算每个时刻搜索图中有效栅格坐标平均值
        p_mean = d[f]/c[f]
        # 4.计算标准差
        p = np.array(p)
        p = p - p_mean
        total = 0
        for kk in range(len(p)):
            total = total + p[kk]
        if c[f] != 0:
            r[f] = math.sqrt(total**2/c[f])
    return t, r


if __name__ == '__main__':
    # 1.加载数据路径文件
    file_path = 'E:/博士论文试验数据/chapter4/zigzag2/2/1604818090/'
    # 2.统计超过指定执行度的栅格数目
    probability_value = 0.5
    time, deviation = run(probability_value, file_path)
    print(deviation)
    print(time)
    # 3.建立输出路径，保存数据
    out_file_path = file_path.split("/")[0] + '/' + file_path.split("/")[1] + '/' + file_path.split("/")[2] + '/' + \
                    'stander_deviation' + '/' + file_path.split("/")[4] + '/'
    makedir.mkdir(out_file_path)
    out_file_name = out_file_path + 'stander_deviation' + '_j' + '.csv'
    result = pd.DataFrame({'time': time, 'stander_deviation': deviation})
    result.to_csv(out_file_name, index=None, encoding='utf8')


