# _*_ coding:utf-8 _*_
# 开发人员：103中山分队-苗润龙
# 开发时间：2020/9/1015:30
# 文件名：stream.py
# 开发工具：PyCharm
# 功能：流线图

from pylab import meshgrid,  arange, streamplot, show
import numpy as np


x, y = meshgrid(arange(-400, 800, 1), arange(-400, 800, 1))

vx = (-x*(x**2+y**2-300**2)-2*y*np.sqrt(x**2+y**2)*300)/(np.sqrt(x**2+y**2)*(x**2+y**2+300**2))
vy = (-y*(x**2+y**2-300**2)+2*x*np.sqrt(x**2+y**2)*300)/(np.sqrt(x**2+y**2)*(x**2+y**2+300**2))

streamplot(x, y, vx, vy, 1)

show()