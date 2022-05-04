import open3d as o3d
import numpy as np

# 引入第三方库

print("Open3D read Point Cloud")
# 输出提示语句
pcd = o3d.io.read_point_cloud(r"milk.pcd")
pcd.paint_uniform_color([0.5, 0.5, 0.5])
# 读取模型文件

pcd_tree = o3d.geometry.KDTreeFlann(pcd)
pcd.colors[100] = [1, 0, 0]

[k, idx, _] = pcd_tree.search_knn_vector_3d(pcd.points[100], 100)
# 确定中心
np.asarray(pcd.colors)[idx[1:], :] = [0, 1, 0]

o3d.visualization.draw_geometries([pcd], width=1200, height=1000)
# 绘制点云模型