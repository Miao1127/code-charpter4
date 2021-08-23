# _*_ coding:utf-8 _*_
# 103中山分队
# 开发人员：Miao
# 开发时间：2019/12/816:27
# 文件名：read_csv.py
# 开发工具：PyCharm
# 功能：读取栅格信息的csv文件

import pandas as pd
import numpy as np


def run(file_name):
    """
    读取CSV文件，并将其转化为列表数据
    :param file_name:
    :return:
    """
    csv_data = pd.read_csv(file_name, header=None)  # 从表格第一行开始读取训练数据
    d = csv_data.values.tolist()
    return d


if __name__ == '__main__':
    # 测试，读取地图矩阵
    data = run("matrix_map.csv")
    data = np.array(data)
    print(data[3, 3])
