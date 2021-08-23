# _*_ coding:utf-8 _*_
# 开发人员：103中山分队-苗润龙
# 开发时间：2020/1/1512:31
# 文件名：heatmap.py
# 开发工具：PyCharm
# 功能：测试热力图

import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
import random


np.random.seed(0)
x = np.zeros((100, 200))
rows, cols = x.shape
for i in range(1, rows-1):
    for j in range(1, cols-1):
        print(i, j)
        x[i][j] = random.uniform(0.05, 0.6)
print(x)


f, ax = plt.subplots()
sns.heatmap(x, annot=False, fmt='.1f', ax=ax)
# plt.axis("equal")
plt.show()
