# _*_ coding:utf-8 _*_
# 103中山分队
# 开发人员：Miao
# 开发时间：2019/12/816:31
# 文件名：zone2csv.py
# 开发工具：PyCharm
# 功能：将每个分区的栅格信息分别写入单独的csv文件

import pandas as pd
import time
import numpy as np

now = time.strftime("%Y-%m-%d-%H_%M_%S", time.localtime(time.time()))  # 用于给输出文件加时间戳


def z2c(zone_dict, file_name):
    """
    将分区写入CSV文件中
    :param file_name: 文件名
    :param zone_dict: 分区字典
    :return: 无返回值
    """
    # 将分区结果按照分区编号顺序写入文件
    for key in zone_dict:
        zone_dict[key] = np.array(zone_dict[key])
    zone_dict['#1'] = zone_dict['#1'].tolist()  # 因为json不认numpy的array，所以需要转化一下
    zone = pd.DataFrame(zone_dict['#1'], index=None, columns=['row', 'col', 'prob'])
    for i in range(1, len(zone_dict)):
        key = '#' + str(i + 1)
        zone_dict[key] = zone_dict[key].tolist()  # 因为json不认numpy的array，所以需要转化一下
        z = pd.DataFrame(zone_dict[key], index=None, columns=['row', 'col', 'prob'])
        zone = pd.concat([zone, z], axis=1)
    zone.to_csv('./data/' + file_name + '_' + now + '.csv', index=None)