# _*_ coding:utf-8 _*_
# 开发人员：103中山分队-苗润龙
# 开发时间：2019/12/914:25
# 文件名：zone2zone.py
# 开发工具：PyCharm
# 功能：分区顶点间的最短距离
import numpy as np
from distance import path_distance
from file_operator import read_json, read_csv
from solver import a_star


def run(vertex_json, map_csv):
    """
    计算各个分区顶点间的最短距离
    :param vertex_json: 分区顶点字典json文件
    :param map_csv: 栅格地图
    :return: 分区距离矩阵
    """
    vertex_dict = read_json.run(vertex_json)
    map_data = np.array(read_csv.run(map_csv))
    # 为保持矩阵的第i行j列对应第#i和第#j分区序号一致，需要+1
    z_distance = np.zeros((len(vertex_dict) + 1, len(vertex_dict) + 1))
    for k in range(len(vertex_dict)):
        if k + 2 > len(vertex_dict):  # 判断key_2是否超出字典包含键值对的数量
            break
        else:
            key_1 = '#' + str(k + 1)
            for kk in range(k + 2, len(vertex_dict) + 1):
                key_2 = '#' + str(kk)
                p_distance = []
                for i in range(len(vertex_dict[key_1])):
                    for j in range(len(vertex_dict[key_2])):
                        start = [int(float(vertex_dict[key_1][i][0])), int(float(vertex_dict[key_1][i][1]))]
                        goal = [int(float(vertex_dict[key_2][j][0])), int(float(vertex_dict[key_2][j][1]))]
                        a = a_star.AStar(map_data, start, goal, 100000)
                        a.run()
                        path = a.path_backtrace()
                        p_distance.append(path_distance.run(path[0], path[1]))  # path[0]存储路径行号, path[1]存储路径列号
                z_distance[k + 1][kk] = min(p_distance)
                z_distance[kk][k + 1] = z_distance[k + 1][kk]
                print(key_1, key_2, z_distance[k + 1][kk])
                print("****************************************")
    np.savetxt('zone2zone_distance_mat.csv', z_distance, delimiter=',')
    return z_distance


if __name__ == '__main__':
    # 测试
    result = run('../zone_operator/vertex_dict.json', '../file_operator/matrix_map_data.csv')
    print(result)
