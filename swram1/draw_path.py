# _*_ coding:utf-8 _*_
# 开发人员：103中山分队-苗润龙
# 开发时间：2020/11/222:06
# 文件名：draw_path.py
# 开发工具：PyCharm
# 功能：绘制USV编队轨迹
from file_operator import read_json, read_csv
import matplotlib.pyplot as plt
from mapping import zone_plot


def draw_animation(path_data):
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
    for key in path_data:
        plt.plot(path_data[key][1:, 0], path_data[key][1:, 1], color_dict[key] + '-')
        plt.plot(path_data[key][-1][0], path_data[key][-1][1],  color_dict[key]+'.')        # 终点
        plt.text(path_data[key][-1][0], path_data[key][-1][1], key, fontdict={
            'family': 'Times New Roman', 'weight': 'normal', 'size': 25})                   # 在终点添加USV编号
        plt.plot(path_data[key][0][0], path_data[key][0][1], 'k*')   # 绘制各个usv起始位置
    plt.show()


if __name__ == '__main__':
    path = read_json.run('E:/博士论文试验数据/chapter5/swarm1/path/1604726609/251usv_path.json')  # USV编队从左下角出发以zigzag形式实施搜索任务
    map_data = read_csv.run('../file_operator/map_data_100_test.csv')   # 区域地图
    draw_animation(path)
