# _*_ coding:utf-8 _*_
# 时间：2020/4/14 21:10
# 作者：苗润龙
# 功能：usv探测模型，根据目标栅格与探测传感器的距离和角度计算传感器对目标栅格的探测确定度
# 重要提示：需要调整探测传感器的性能参数alpha,beta以及高斯噪声的参数sigma，本程序中设置为默认值，
# 减小alpha值可以提高传感器纵向感测确定度，减小beta值可以提高传感器横向观测确定度

import numpy as np
from matplotlib import pyplot as plt


def sensor(mm, nn, x, y, phi, d, r_max, alpha=0.85, beta=0.65, sigma=0.02):
    """
    usv探测模型
    :param mm: 目标栅格的列号
    :param nn: 目标栅格的行号
    :param x: usv在X轴坐标位置
    :param y: usv在Y轴坐标位置
    :param phi: usv航向角，弧度值
    :param d: 栅格尺寸
    :param r_max: 传感器最大探测范围
    :param alpha: 探测传感器性能参数
    :param beta: 探测传感器性能参数
    :param sigma: 噪声的标准差
    :return: 返回目标栅格的探测概率
    """
    xmn = (mm - 0.5) * d
    ymn = (nn - 0.5) * d
    distance = np.sqrt((x - xmn) ** 2 + (y - ymn) ** 2)
    if phi < 0 and ymn - y == 0 and xmn < x:
        angle = -np.pi
    else:
        angle = np.arctan2(ymn - y, xmn - x)
    # 需要判断探测区域是否覆盖-180度射线
    if phi < -np.pi/2 and angle > 0:
        diff = angle - phi - 2*np.pi
    elif phi > np.pi/2 and angle < 0:
        diff = 2*np.pi - phi + angle
    else:
        diff = angle - phi
    p = (2/(1+np.exp(alpha*distance-r_max)) - 1) * np.cos(beta*diff)
    # 修正
    if p < 0.5 and np.pi/6 >= diff >= -np.pi/6:
        p = 0.5
    elif p < 0.4 and -np.pi/6 > diff >= -np.pi/3:
        p = 0.4
    elif p < 0.4 and np.pi/3 >= diff > np.pi/6:
        p = 0.4
    elif p < 0.3 and -np.pi/3 > diff >= -np.pi/2:
        p = 0.3
    elif p < 0.3 and np.pi/2 >= diff > np.pi/3:
        p = 0.3
    pb = np.random.normal(loc=0, scale=sigma, size=None)
    p = p + pb
    # 防止饱和
    if p > 1:
        p = 0.9999
    elif p < 0:
        p = 0.1
    return p


if __name__ == '__main__':
    # 单个值测试
    pro = sensor(7, 3, 5.5, 5.5, 0, 1, 5, 0.1, 0.8, 0.1)
    print(pro)
