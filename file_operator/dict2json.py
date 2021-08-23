# _*_ coding:utf-8 _*_
# 103中山分队
# 开发人员：Miao
# 开发时间：2019/12/816:25
# 文件名：dict2json.py
# 开发工具：PyCharm
# 功能：将字典保存到json文件中

import json
import numpy as np


def run(dict_name, file_name):
    # 将分区结果写入json文件
    for key in dict_name:
        dict_name[key] = np.array(dict_name[key]).tolist()  # 因为json不认numpy的array，所以需要转化一下
    json_str = json.dumps(dict_name)
    with open(file_name + '.json', 'w') as f1:
        f1.write(json_str)


if __name__ == '__main__':
    # 测试
    a = {'#1': np.array([[1, 3, 0.5], [2, 3, 0.5]]),
         '#2': np.array([[3, 5, 0.5], [1, 3, 0.5]])}
    run(a, 'test_dict2json')
