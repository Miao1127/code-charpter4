# _*_ coding:utf-8 _*_
# 开发人员：103中山分队-苗润龙
# 开发时间：2019/12/910:41
# 文件名：image2matrix.py
# 开发工具：PyCharm
# 功能：将jpg任务区域图像转换为矩阵数据

import numpy as np
import cv2


def i2m(image_name):
    """
    将图像转化为矩阵数据
    :param image_name: 图像地址
    :return: 以图像高度像素数为行数，以图像宽度像素数为列数的矩阵
    """
    img = cv2.imread(image_name, cv2.IMREAD_GRAYSCALE)

    # img1 = cv2.flip(img, 0)                  # 将图像上下翻转
    new_data = np.array(img, dtype='float')    # 存储成浮点型numpy的数组,0是白，255是黑

    # 将灰度图中的信息转化为概率信息
    rows, cols = new_data.shape
    for i in range(rows):
        for j in range(cols):
            if new_data[i][j] > 180:
                new_data[i][j] = 0.5
            else:
                new_data[i][j] = 0
    np.savetxt('matrix_map.csv', new_data, delimiter=',')
    print("图像数据存储为CSV文件！")
    print("栅格地图的行数和列数分别为%d和%d." % (rows, cols))
    print("将彩色图像转为灰度图并转化为数据矩阵！")
    return new_data


if __name__ == '__main__':
    # 算例
    print("将图像转化成数据")
    data = i2m('map.jpg')
    print(data.shape)
