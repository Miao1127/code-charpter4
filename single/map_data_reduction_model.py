# _*_ coding:utf-8 _*_
# 时间：2020/5/6 19:38
# 作者：苗润龙
# 功能：概率地图衰减模型

import time
import numpy as np


def map_data_model(data):
    while True:
        time.sleep(t_map)
        if data != 0:
            data = 0.99999*data + np.random.normal(loc=0, scale=0.001, size=None)
            if 0 < data < 0.1:
                data = 0.1
        print(data)


if __name__ == '__main__':
    t_map = 0.1
    map_data_model(0.9)
