import open3d as o3d
import numpy as np

# *******************************凸包***********************************
pcd = o3d.io.read_point_cloud("../test_data/fragment.ply")  # 读取ply或者pcd文件
# 裁剪点云
vol = o3d.visualization.read_selection_polygon_volume(
    "../test_data/Crop/cropped.json")
chair = vol.crop_point_cloud(pcd)

# 计算点云的凸包
hull, _ = chair.compute_convex_hull()
hull_ls = o3d.geometry.LineSet.create_from_triangle_mesh(hull)
hull_ls.paint_uniform_color((1, 0, 0))  # 凸包的颜色
o3d.visualization.draw_geometries([chair, hull_ls])
