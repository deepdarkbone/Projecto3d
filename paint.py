import open3d as o3d
import numpy as np

# 引入第三方库

print("Open3D read Point Cloud")
# 输出提示文字
pcd = o3d.io.read_point_cloud("pcl_logo.pcd")
pcd.paint_uniform_color([0.5, 0.5, 0.5])
# 读取模型
print(pcd)

o3d.visualization.draw_geometries([pcd], width=800, height=600)
# 绘制生成点云

