# _*_ coding:utf-8 _*_
# 开发人员：103中山分队-苗润龙
# 开发时间：2020/7/1415:09
# 文件名：color_test.py
# 开发工具：PyCharm
# 功能：测试颜色
import matplotlib.pyplot as plt

color_dict = {'#1': 'm', '#2': 'y', '#3': 'g', '#4': 'b', '#5': 'c', '#6': 'r', '#7': 'brown', '#8': 'maroon',
              '#9': 'coral', '#10': 'peru', '#11': 'orange', '#12': 'olive', '#13': 'crimson', '#14': 'darkgreen',
              '#15': 'slategrey', '#16': 'tomato', '#17': 'dodgerblue', '#18': 'navy', '#19': 'violet',
              '#20': 'deeppink', '#21': 'pink'}  # 各个usv轨迹颜色设置

fig = plt.figure()



x = range(100)
y = [i**2 for i in x]
values = color_dict.values()
for value in values:
    plt.plot(x, y, linewidth='1', label=value, color=value, linestyle=':', marker='|')
    plt.legend(loc='upper left')
plt.show()

