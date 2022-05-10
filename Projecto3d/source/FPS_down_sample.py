# FPS下采样

import open3d as o3d
import numpy as np


def fps_filter(point_cloud):
    filtered_points = []
    # 随机选取第一个点当做FPS下采样的起点
    point_first_index = np.random.randint(0, len(point_cloud))
    filtered_points.append(point_cloud[point_first_index])
    # 按照50%作为下采样的目标点数
    downsample_point_num = len(point_cloud) * 0.5
    for i in range(int(downsample_point_num)):
        ipoint_jpoint_distance = []
        if(i == 0):     # 使用随机选取的点作为FPS的第一个点
            i_x = point_cloud[point_first_index][0]
            i_y = point_cloud[point_first_index][1]
            i_z = point_cloud[point_first_index][2]
            for j in range(len(point_cloud)):
                j_x = point_cloud[j][0]
                j_y = point_cloud[j][1]
                j_z = point_cloud[j][2]
                distance = pow((i_x-j_x), 2) + pow((i_y-j_y), 2) + pow((i_z-j_z), 2)
                ipoint_jpoint_distance.append(distance)
            distance_sort = np.argsort(ipoint_jpoint_distance)
            filtered_points.append(point_cloud[distance_sort[-1]])
            continue
        # 遍历点云中的每一个点
        for j in range(len(point_cloud)):
            j_x = point_cloud[j][0]
            j_y = point_cloud[j][1]
            j_z = point_cloud[j][2]
            distance_list = []
            # 计算遍历到的原点云中的点与已采到的点之间的距离
            for k in range(len(filtered_points)):
                point_repeat = True     # point_repeat防止比较同一个点之间的距离
                k_x = filtered_points[k][0]
                k_y = filtered_points[k][1]
                k_z = filtered_points[k][2]
                if (j_x == k_x and j_y == k_y and j_z == k_z):
                    point_repeat = False
                    break
                distance = pow((i_x-j_x), 2) + pow((i_y-j_y), 2) + pow((i_z-j_z), 2)
                distance_list.append(distance)
            if point_repeat is True:
                distance_avg = np.mean(distance_list)
                ipoint_jpoint_distance.append(distance_avg)
        distance_sort = np.argsort(ipoint_jpoint_distance)          # 对距离排序，返回索引序号
        filtered_points.append(point_cloud[distance_sort[-1]])      # 将具有最大距离对应的点加入filtered_points
    print(len(filtered_points))
    # 把点云格式改成array，并对外返回
    filtered_points = np.array(filtered_points, dtype=np.float64)
    return filtered_points


def main():
    pcd = o3d.io.read_point_cloud('test9.pcd')
    o3d.visualization.draw_geometries([pcd], window_name="滤波前的点云",
                                      width=1024, height=768,
                                      left=50, top=50,
                                      mesh_show_back_face=False)

    # 调用FPS滤波函数，实现滤波
    filtered_cloud = fps_filter(pcd.points)
    point_cloud_out = o3d.geometry.PointCloud()
    point_cloud_out.points = o3d.utility.Vector3dVector(filtered_cloud)
    o3d.io.write_point_cloud("copy_of_fragment.pcd", point_cloud_out)
    # 显示滤波后的点云
    o3d.visualization.draw_geometries([pcd], window_name="FPS滤波后的点云",
                                      width=1024, height=768,
                                      left=50, top=50,
                                      mesh_show_back_face=False)


if __name__ == '__main__':
    main()


