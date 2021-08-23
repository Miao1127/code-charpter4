# _*_ coding:utf-8 _*_
# 开发人员：103中山分队-苗润龙
# 开发时间：2019/12/2013:25
# 文件名：nested_dict2json.py
# 开发工具：PyCharm
# 功能：存储嵌套字典到json中


import json
import numpy as np


def run(dict_name, file_name):
    # 将分区结果写入json文件
    for key_1 in dict_name:
        for key_2 in dict_name[key_1]:
            dict_name[key_1][key_2] = np.array(dict_name[key_1][key_2]).tolist()  # 因为json不认numpy的array，所以需要转化一下
    json_str = json.dumps(dict_name)
    with open(file_name + '.json', 'w') as f1:
        f1.write(json_str)


if __name__ == '__main__':
    # 测试
    a = {'##1': {'#1': np.array([[1, 3, 0.5], [2, 3, 0.5]]),
         '#2': np.array([[3, 5, 0.5], [1, 3, 0.5]])}}
    run(a, 'test_dict2json')
