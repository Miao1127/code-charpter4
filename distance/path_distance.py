# _*_ coding:utf-8 _*_
# 开发人员：103中山分队-苗润龙
# 开发时间：2019/12/914:20
# 文件名：path_distance.py
# 开发工具：PyCharm
# 功能：计算路径长度

import math


def run(x, y):
    """
    计算usv在栅格中移动的栅格距离
    :param x: 栅格列号
    :param y: 栅格行号
    :return: 栅格距离，栅格边长定义为单位距离，实际距离需要用栅格距离乘以栅格地图的实际边长
    """
    distance = 0
    if len(x) == len(y):  # 判断栅格的所有行号是否与列号数量相同
        n = len(x)
        for i in range(n):  # 遍历每个栅格
            if i + 2 > n:   # 判断是否超出栅格数目
                break
            elif x[i] != x[i + 1] or y[i] != y[i + 1]:    # 排除前后两个栅格编号相同
                if x[i] == x[i + 1] or y[i] == y[i + 1]:  # 只要列号或者行号有一个不改变，说明usv移动的方向为x方向或者y方向
                    distance += 1
                else:
                    distance += math.sqrt(2)
    return distance


if __name__ == '__main__':
    # 测试
    x_path = [1, 1, 1, 1, 2, 3]
    y_path = [1, 1, 2, 2, 2, 3]
    result = run(x_path, y_path)
    print(result)
