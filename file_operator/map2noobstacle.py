# 时间： 21:27
# 作者：苗润龙
# 功能：将带有障碍物的搜索区域地图转化为无障碍物搜索区域

from file_operator import read_csv
import numpy as np
import random
import scipy.stats
import math

dist = scipy.stats.multivariate_normal(mean=[30, 30], cov=[[0.01, 0], [0.01, 1]])
x = read_csv.run('../file_operator/map_data_100.csv')
rows, cols = np.array(x).shape
for i in range(rows):
    for j in range(cols):
        if x[i][j] != 0:
            # x[i][j] = random.uniform(0, 0.5)     # 随机概率
            x[i][j] = 0.1                        # 固定概率值
        # if math.sqrt((i-30)**2+(j-30)**2) < 10:  # 加入高斯分布概率
        #     x[i][j] = dist.pdf([i, j])
np.savetxt('map_data_100_test2.csv', x, delimiter=',')
# print(x)


