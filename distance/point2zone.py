# _*_ coding:utf-8 _*_
# 开发人员：103中山分队-苗润龙
# 开发时间：2019/12/1217:00
# 文件名：point2zone.py
# 开发工具：PyCharm
# 功能：usv起始位置到各个分区顶点的最短距离，因为分区顶点有多个，选取最短者作为参考,相应顶点即分区遍历进入点，此程序也可以实现对多个usv
# 和多个分区之间的距离计算
# 路径存储分析（若实现多个usv和多个分区之间距离计算，以5个usv从初始位置到18个分区顶点的最短路径为例）：1.需要存储的路径有5*18个；
# 2.每个路径的数据量不同，其形式为n行三列的二维矩阵；3.每个路径由一系列有序栅格构成；4.栅格的格式为[1, 2, 0.5]。所以采用字典的嵌
# 套形式来记录路径
# 修改记录：2019.12.14 增加占据分区，由于占据分区不需要参与BPSO分区分配，所以可以无需再计算距离；
#          2019.12.15 usv与各个分区的距离放在各个usv上单独计算，计算完成后再相互交换；

import numpy as np
from file_operator import read_json
from file_operator import read_csv, nested_dict2json
from solver import a_star
from distance import path_distance


def run(usv_dict, occupied_zone, vertex_dict, map_data):
    """
    计算当前usv到各个分区顶点的最短距离
    :param usv_dict:      当前usv坐标位置，数据形式：{"##1": ["#0", 0, 1, 0.25, 11, 5]}
    :param occupied_zone: 已被分配过的分区编号列表，[4, 5, 6, 7, 8, 9]
    :param vertex_dict:  分区顶点字典
    :param map_data:     栅格地图
    :return:            usv到达各个分区最短距离矩阵和相应路径
    """
    map_data = np.array(map_data)
    z_distance = np.zeros((1, len(vertex_dict) + 1))                    # 存储距离
    all_path = {}                                                       # 存储路径
    temp = float('inf')
    for k in range(len(usv_dict)):
        key_1 = '##' + str(k + 1)    # usv编号
        while key_1 not in usv_dict:
            k += 1
            key_1 = '##' + str(k + 1)  # usv编号
        z_distance[0][0] = k + 1       # 记录usv编号
        all_path[key_1] = {}
        for kk in range(len(vertex_dict)):
            key_2 = '#' + str(kk + 1)
            p_distance = []
            if kk+1 in occupied_zone:
                p_distance = [0]
                all_path[key_1][key_2] = [[]]
            else:
                for j in range(len(vertex_dict[key_2])):
                    start = [int(float(usv_dict[key_1][4])), int(float(usv_dict[key_1][5]))]    # 起点和终点以行列编号输入
                    goal = [int(float(vertex_dict[key_2][j][0])), int(float(vertex_dict[key_2][j][1]))]
                    a = a_star.AStar(map_data, start, goal, 100000)
                    a.run()
                    path = a.path_backtrace()
                    p_distance.append(path_distance.run(path[0], path[1]))  # path[0]存储路径行号, path[1]存储路径列号
                    if path_distance.run(path[0], path[1]) < temp:
                        all_path[key_1][key_2] = [[]]            # 清空上次记录的路径，否则较长的上次路径后段会遗留在路径中
                        all_path[key_1][key_2] = np.c_[path[0], path[1]]
                        temp = path_distance.run(path[0], path[1])
            temp = float('inf')  # 重置，用于下次比较
            z_distance[0][kk + 1] = min(p_distance)  # ＋1是为了保证z_distance矩阵中元素编号和分区编号保持一致
            print(key_1, key_2, z_distance[0][kk + 1])
            print("****************************************")
    return z_distance, all_path


if __name__ == '__main__':
    # 计算各个的usv从出发点到各个分区的最短距离
    usv_json = '../zone_operator/usv1_start_position.json'
    v_json = '../zone_operator/vertex_dict.json'
    m_csv = "../file_operator/matrix_map_data_add_start.csv"
    u_dict = read_json.run(usv_json)   # 数据形式：{"##1": [0, 0, 1, 0.25, 12, 6]}
    v_dict = read_json.run(v_json)
    m_data = read_csv.run(m_csv)
    o_zone_list = [4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18]
    # o_zone_list = []

    print('*'*20)
    print('输入量：')
    print('usv_dict: %s' % u_dict)
    print('occupied_zone: %s' % o_zone_list)
    print('vertex_dict: %s' % v_dict)
    print('map_data: %s' % m_data)
    print('*'*20)
    u2z_mat, s_path = run(u_dict, o_zone_list, v_dict, m_data)
    np.savetxt('u2z_usv1_distance.csv', u2z_mat, delimiter=',')
    nested_dict2json.run(s_path, 's_path')

    print('!'*20)
    print('输出量：')
    u2z_d = list(u2z_mat[0])
    print('u2z_mat:  %s' % u2z_d)
    print('s_path: %s' % s_path)
