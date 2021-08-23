# _*_ coding:utf-8 _*_
# 开发人员：103中山分队-苗润龙
# 开发时间：2019/12/1416:56
# 文件名：zone2zone_v2.py
# 开发工具：PyCharm
# 功能：各个分区形心间距离

import numpy as np
import math
from file_operator import read_json


def run(zone_dict):
    distance_mat = np.zeros((len(z_dict) + 1, len(zone_dict) + 1))
    for k in range(len(z_dict)):
        key_1 = '#' + str(k + 1)
        x_1 = sum(zone_dict[key_1][:, 0]) / len(zone_dict[key_1])
        y_1 = sum(zone_dict[key_1][:, 1]) / len(zone_dict[key_1])
        for kk in range(k + 2, len(zone_dict) + 1):
            key_2 = '#' + str(kk)
            x_2 = sum(zone_dict[key_2][:, 0]) / len(zone_dict[key_2])
            y_2 = sum(zone_dict[key_2][:, 1]) / len(zone_dict[key_2])
            distance_mat[k + 1][kk] = math.sqrt((x_1 - x_2)**2 + (y_1 - y_2)**2)
            distance_mat[kk][k + 1] = distance_mat[k + 1][kk]
    np.savetxt('z2z_distance_v2.csv', distance_mat, delimiter=',')
    return distance_mat


if __name__ == '__main__':
    # 测试
    z_dict = read_json.run('../zone_operator/split_dict.json')
    # z_dict = {'#1': np.array([[1, 1], [1, 2], [2, 1], [2, 2]]), '#2': np.array([[4, 1], [4, 2]])}
    d_mat = run(z_dict)
    print(d_mat)
