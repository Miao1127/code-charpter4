# _*_ coding:utf-8 _*_
# 时间：2020/4/17 14:12
# 作者：苗润龙
# 功能：图形化显示传感器探测数据模型

# _*_ coding:utf-8 _*_
# 时间：2020/4/15 20:29
# 作者：苗润龙
# 功能：测试传感器模型

import numpy as np
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# 定义坐标轴
fig = plt.figure()
ax = plt.axes(projection='3d')

# 生成三维数据
xx = np.arange(-6, 6, 0.1)
yy = np.arange(0, 6, 0.1)
X, Y = np.meshgrid(xx, yy)
distance = np.sqrt(X**2+Y**2)
diff = np.arctan2(X, Y)
Z = (2/(1+np.exp(0.85*distance-5)) - 1) * np.cos(0.65*diff)
row, col = Z.shape
for i in range(row):
    for j in range(col):
        if Z[i][j] < 0 and 30 > diff[i][j] > -30:
            Z[i][j] = 0



# 作图
surf = ax.plot_surface(X, Y, Z, alpha=1, cmap='winter')     # 生成表面， alpha 用于控制透明度
# ax.contour(X, Y, Z, zdir='z', offset=-3, cmap="rainbow")  # 生成z方向投影，投到x-y平面
ax.contour(X, Y, Z, zdir='x', offset=-6, cmap="rainbow")  # 生成x方向投影，投到y-z平面
ax.contour(X, Y, Z, zdir='y', offset=6, cmap="rainbow")   # 生成y方向投影，投到x-z平面
# ax.contourf(X, Y, Z, zdir='y', offset=6, cmap="rainbow")   # 生成y方向投影填充，投到x-z平面，contourf()函数
fig.colorbar(surf, shrink=0.5, aspect=5)

# 设定显示范围
# ax.set_xlabel('X')
# ax.set_xlim(-6, 4)  # 拉开坐标轴范围显示投影
# ax.set_ylabel('Y')
# ax.set_ylim(-4, 6)
# ax.set_zlabel('Z')
# ax.set_zlim(-3, 3)
print(Z)
np.savetxt('data.csv', Z.T, delimiter=',')
plt.show()
