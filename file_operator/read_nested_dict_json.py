# _*_ coding:utf-8 _*_
# 开发人员：103中山分队-苗润龙
# 开发时间：2019/12/2021:27
# 文件名：read_nested_dict_json.py
# 开发工具：PyCharm
# 功能：读取嵌套字典的json文件

import json


def run(file_name):
    # 测试分区拆分
    with open(file_name, mode='r', encoding='gbk') as f2:  # 读取文件中的分区字典
        z_dict = json.load(f2)
    for key in z_dict:  # 需要重新转换为numpy形式的array
        z_dict[key] = z_dict[key]
    return z_dict


if __name__ == '__main__':
    # 测试
    data = run('../usv_model/usv1_path.json')
    print(data)
