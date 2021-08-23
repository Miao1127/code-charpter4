# _*_ coding:utf-8 _*_
# 开发人员：103中山分队-苗润龙
# 开发时间：2020/11/415:21
# 文件名：cover_rate.py
# 开发工具：PyCharm
# 功能：计算覆盖率

from file_operator import read_csv, makedir
import os
import numpy as np
import pandas as pd


def run(p_value, f_path):
    for i, j, k in os.walk(f_path):
        print(k)
    t = np.zeros(len(k))
    c = np.zeros(len(k))
    for f in range(len(k)):
        t[f] = int(k[f].split("#")[0])
        map_data = np.array(read_csv.run(f_path + k[f]))
        row, col = map_data.shape
        for i in range(row):
            for j in range(col):
                if map_data[i][j] >= p_value:
                    c[f] += 1
    return t, c


if __name__ == '__main__':
    # 1.加载数据路径文件
    file_path = 'E:/博士论文试验数据/chapter4/zigzag2/2/1604818090/'
    # 2.统计超过指定执行度的栅格数目
    probability_value = 0.5
    time, count = run(probability_value, file_path)
    print(count)
    print(time)
    # 3.建立输出路径，保存数据
    out_file_path = file_path.split("/")[0] + '/' + file_path.split("/")[1] + '/' + file_path.split("/")[2] + '/' + \
                    'cover_rate' + '/' + '2020110801' + '/'
    makedir.mkdir(out_file_path)
    out_file_name = out_file_path + 'cover_rate' + file_path.split("/")[3] + '.csv'
    result = pd.DataFrame({'time': time, 'count': count})
    result.to_csv(out_file_name, index=None, encoding='utf8')

