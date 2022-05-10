#计算法向量，绘制并访问

import open3d as o3d
import numpy as np

pcd = o3d.io.read_point_cloud(r"/home/amadeus/PycharmProjects/Projecto3d/pointCloud.ply")
downpcd = pcd.voxel_down_sample(voxel_size=50)
print("Recompute the normal of the downsampled point cloud")
#法向量估计的两个参数，radius为搜索半径，max_nn为考虑的邻居数量
downpcd.estimate_normals(
    search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=500, max_nn=30))

o3d.visualization.draw_geometries([downpcd],point_show_normal=True)

# Access estimated vertex normal  访问定点的法向量
print("Print a normal vector of the 0th point")
print(downpcd.normals[0])
# Normal vectors can be transformed as a numpy array using np.asarray.
print("Print the normal vectors of the first 10 points")
print(np.asarray(downpcd.normals)[:10, :])
