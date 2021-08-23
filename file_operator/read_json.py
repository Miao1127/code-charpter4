# _*_ coding:utf-8 _*_
# 103中山分队
# 开发人员：Miao
# 开发时间：2019/12/816:29
# 文件名：read_json.py
# 开发工具：PyCharm
# 功能：读取存储字典的json文件

import json
import numpy as np


def run(file_name):
    # 测试分区拆分
    with open(file_name, mode='r', encoding='gbk') as f2:  # 读取文件中的分区字典
        z_dict = json.load(f2)
    for key in z_dict:  # 需要重新转换为numpy形式的array
        z_dict[key] = np.array(z_dict[key])
    return z_dict


if __name__ == '__main__':
    # 测试
    data = run('test_dict2json.json')
    print(data)