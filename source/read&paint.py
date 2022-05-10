#读一个点云,可视化
#对其进行体素下采样，可视化

import open3d as o3d
import numpy as np

# 按 Ctrl+F8 切换断点。
print("Load a ply point cloud, print it, and render it")
pcd = o3d.io.read_point_cloud(r"/home/amadeus/PycharmProjects/Projecto3d/pointCloud.ply")
print(pcd)                      #简单打印点云信息
print(np.asarray(pcd.points))      #打印点云所有点
o3d.visualization.draw_geometries([pcd])

print("Downsample the point cloud with a voxel of 100")
downpcd = pcd.voxel_down_sample(voxel_size=100)
o3d.visualization.draw_geometries([downpcd])
