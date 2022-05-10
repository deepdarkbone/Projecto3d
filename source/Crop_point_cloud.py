#不使用json文件裁减

import open3d as o3d
import numpy as np

# ----------------------------读取点云并可视化----------------------------
pcd = o3d.io.read_point_cloud("/home/amadeus/PycharmProjects/Projecto3d/Open3D_test_data/bunny.pcd")
print(pcd)
o3d.visualization.draw_geometries([pcd],
                                  window_name="原始点云",
                                  width=800, height=600)
# ------------------------------定义裁剪功能------------------------------
vol = o3d.visualization.SelectionPolygonVolume()
# ------------------------给定任意裁剪范围的顶点坐标-----------------------
bounding_polygon = np.array([
    [1.25, 3.17, 0.25],
    [0.65, -7.21, 0.11],
    [13.6, -40.0, 0.21],
    [42.54, -41.41, 0],
    [63.2, -34.3, 0],
    [77.8, -10.2, 0],
    [25.8, 36.2, 0],
    [6, 27, 0],
    [3, 21, 1],
], dtype=np.float64)
print("裁剪范围顶点坐标为：", bounding_polygon)
vol.bounding_polygon = o3d.utility.Vector3dVector(bounding_polygon)
# --------------------------z轴用于定义所选区域的高度-----------------------
vol.orthogonal_axis = "z"
vol.axis_min = -80  # 设置取值范围的最小值
vol.axis_max = 80   # 设置取值范围的最大值
# ------------------------------裁剪点云----------------------------------
crop_cloud = vol.crop_point_cloud(pcd)
print(crop_cloud)
# -----------------------------结果可视化---------------------------------
o3d.visualization.draw_geometries([crop_cloud],
                                  window_name="裁剪出的点云",
                                  width=800, height=600)

