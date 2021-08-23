# _*_ coding:utf-8 _*_
# 开发人员：103中山分队-苗润龙
# 开发时间：2020/11/28:11
# 文件名：time_test.py
# 开发工具：PyCharm
# 功能：测试时间

import time
import numpy as np
from file_operator import makedir

start_time = time.time()

tic = time.time()
time.sleep(1)
toc = time.time()
map_data = [1, 2, 3]
makedir.mkdir('./data/' + str(int(start_time)))
data = [1, 2, 3]
print(len(data))

print(toc-tic)
print(tic)
print(toc)
print(int(time.time()))
while time.time()-start_time < 10:
    if time.time() - tic > 1:
        np.savetxt('./data/' + str(int(start_time)) + '/' + 'map_data.csv', map_data, delimiter=',')
        print("save map data")
        tic = time.time()
