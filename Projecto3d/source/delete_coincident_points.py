#删除点云中的重合点
import open3d as o3d
import numpy as np


pcd = o3d.io.read_point_cloud("/home/amadeus/PycharmProjects/Projecto3d/Open3D_test_data/cloud_bin_0.pcd")
o3d.visualization.draw_geometries([pcd])
point = np.asarray(pcd.points)
point_size = point.shape[0]
print("原始点云中点的个数为：", point_size)
tree = o3d.geometry.KDTreeFlann(pcd)  # 建立KD树索引
radius = 0.000001                     # 定义搜索半径
total_idx = []
for i in range(point_size):
    [k, idx, _] = tree.search_radius_vector_3d(pcd.points[i], radius)  # 半径搜索
    if k != 0:
        total_idx.append(np.array(idx[0]))
true_idx = np.unique(total_idx)                          # 去除重复点的索引
repe_cloud = pcd.select_by_index(true_idx, invert=True)  # 获取重复的点
true_cloud = pcd.select_by_index(true_idx)               # 获取去重之后的点
print("删除重复点：", repe_cloud)
print("去重之后：", true_cloud)
o3d.io.write_point_cloud("coincidence.pcd", repe_cloud)
o3d.io.write_point_cloud("After.pcd", true_cloud)

