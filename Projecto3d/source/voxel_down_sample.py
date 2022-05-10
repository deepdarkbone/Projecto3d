# 体素下采样

import open3d as o3d
import numpy as np

pcd = o3d.io.read_point_cloud("/home/amadeus/PycharmProjects/Projecto3d/Open3D_test_data/bunny.pcd")
print("原始点云中点的个数为：", np.asarray(pcd.points).shape[0])
# o3d.visualization.draw_geometries([pcd])
print("使用边长为0.005的体素对点云进行下采样")
downpcd = pcd.voxel_down_sample(voxel_size=0.005)
print("下采样之后点的个数为：", np.asarray(downpcd.points).shape[0])
o3d.visualization.draw_geometries([downpcd], window_name="体素滤波",
                                  width=1200, height=800,
                                  left=50, top=50,
                                  mesh_show_back_face=False)

