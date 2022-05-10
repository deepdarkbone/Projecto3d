#最小生成树调整法向量方向
#orient_normals_consistent_tangent_plane(self, k)   k为进行最小代价生成树构建图优化时所用的邻域点的个数

import open3d as o3d
import numpy as np

pcd = o3d.io.read_point_cloud("/home/amadeus/PycharmProjects/Projecto3d/Open3D_test_data/bunny.pcd")
pcd.paint_uniform_color([1.0, 0.0, 0.0])
pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.01, max_nn=30))
pcd.orient_normals_consistent_tangent_plane(10)  # 最小生成树
print(np.asarray(pcd.normals)[:10, :])  # 输出前10个点的法向量
o3d.visualization.draw_geometries([pcd], point_show_normal=True, window_name="朝向相机位置",
                                  width=1024, height=768,
                                  left=50, top=50,
                                  mesh_show_back_face=False)  # 可视化点云和法线

