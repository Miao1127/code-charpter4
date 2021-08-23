# _*_ coding:utf-8 _*_
# 103中山分队
# 开发人员：Miao
# 开发时间：2019/12/816:28
# 文件名：read_excel.py
# 开发工具：PyCharm
# 功能：读取栅格信息的excel文件

import xlrd
import numpy as np


def read_excel(file_path):
    """
    从excel文件中读取数据并返回一个二维数组
    :param file_path: 文件路径
    :return: 二维数组
    """
    workbook = xlrd.open_workbook(file_path)  # 打开文件
    sheet = workbook.sheet_by_index(0)  # 根据sheet索引或者名称获取sheet内容，sheet索引从0开始

    # 存储到一个二维数组中
    my_data = []
    temp = []
    for i in range(sheet.nrows):
        for j in range(sheet.ncols):
            temp.append(sheet.cell(i, j).value)
        my_data.append(temp)
        temp = []

    return np.array(my_data)    # 调试本程序时，没有将data转化为np.array()的形式，导致报错
