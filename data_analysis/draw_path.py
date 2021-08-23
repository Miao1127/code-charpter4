# _*_ coding:utf-8 _*_
# 开发人员：103中山分队-苗润龙
# 开发时间：2020/11/610:33
# 文件名：draw_path.py
# 开发工具：PyCharm
# 功能：批量绘制轨迹图
from file_operator import read_json, read_csv, makedir
import matplotlib.pyplot as plt
from mapping import zone_plot
import os
import numpy as np


def draw_animation(p_data, s_path, t_position_x, t_position_y):
    """
    每次画面刷新即把所有的位置和轨迹重新画一遍
    :return:
    """
    global map_data
    usv_position = {}        # 记录各个usv的当前位置
    color_dict = {'#1': 'm', '#2': 'y', '#3': 'g', '#4': 'b', '#5': 'c'}  # 各个usv轨迹颜色设置
    # 创建一个字典，用于存储各个usv的当前位置信息和轨迹信息，格式为：usv_dict = {'#1'：[[x, y],  [x_tra, y_tra]]}
    # 当前位置的更新方式为对usv_dict['#1'][0]赋值，轨迹更新方式为对usv_dict['#1']追加元素
    fig = plt.figure('编队轨迹图', figsize=(10, 10))
    ax = fig.add_subplot(111)
    # 加载地图数据
    ox, oy = zone_plot.load_map(map_data)
    plt.cla()              # 清除当前图形中的当前活动轴，其他轴不受影响
    # 绘制地图障碍物信息
    # ax.invert_yaxis()      # y轴坐标刻度从上到下递增
    # ax.xaxis.tick_top()    # x坐标轴置于图像上方
    plt.plot(ox, oy, ".k")
    plt.tick_params(labelsize=25)
    labels = ax.get_xticklabels() + ax.get_yticklabels()
    [label.set_fontname('Times New Roman') for label in labels]
    plt.xlabel('x(m)', fontdict={'family': 'Times New Roman', 'weight': 'normal', 'size': 25})
    plt.ylabel('y(m)', fontdict={'family': 'Times New Roman', 'weight': 'normal', 'size': 25})
    plt.grid(True)
    plt.axis("equal")

    # 绘制各个usv位置和轨迹
    try:
        for key in p_data:
            plt.plot(p_data[key][1:, 0], p_data[key][1:, 1], color_dict[key] + '-')
            plt.plot(p_data[key][-1][0], p_data[key][-1][1], color_dict[key] + '.')  # 终点
            plt.text(p_data[key][-1][0], p_data[key][-1][1], key, fontdict={
                'family': 'Times New Roman', 'weight': 'normal', 'size': 25})  # 在终点添加USV编号
            plt.plot(p_data[key][0][0], p_data[key][0][1], 'k*')  # 绘制各个usv起始位置
        # 绘制静态和动态目标点
        plt.scatter(t_position_x, t_position_y, s=100, marker='^', c='k')  # 绘制静态目标点
        plt.scatter(np.random.randint(10, 95, size=10), np.random.randint(10, 95, size=10),
                    s=100, marker='o', c='k')  # 绘制动态目标点
        plt.savefig(s_path)
        # plt.pause(0.1)
        plt.close()
        # plt.show()

    except Exception as err:
        print('err:')
        print(err)


if __name__ == '__main__':
    # 1.读入地图和静态目标点位置
    map_data = read_csv.run('../file_operator/map_data_100_test2.csv')  # 区域地图
    target_position_x = [50, 18, 90, 18, 78, 85, 50, 49, 15, 39]
    target_position_y = [50, 85, 90, 25, 22, 69, 83, 18, 58, 40]
    # 2.批量导入文件路径
    file_path = "E:/博士论文试验数据/chapter5/zigzag1/path/1607057106/"
    save_path = "E:/博士论文试验数据/chapter5/zigzag1/path_figure/" + file_path.split("/")[4] + '/'
    makedir.mkdir(save_path)
    for i, j, k in os.walk(file_path):  # 读入文件夹中文件名称
        print(k)

    # 3.绘制轨迹图并保存
    for f in tuple(k):
        path_data = read_json.run(file_path + f)  # USV编队从左下角出发以zigzag形式实施搜索任务
        time = f.split('#')[0]
        file_name = save_path + time + '.png'
        draw_animation(path_data, file_name, target_position_x, target_position_y)

