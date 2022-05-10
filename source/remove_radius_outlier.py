# 半径滤波器
#nb_points (int):选择球体中最少点的数量
#Radius(float):用来计算点的邻域数量的球的半径

import open3d as o3d
pcd = o3d.io.read_point_cloud("/home/amadeus/PycharmProjects/Projecto3d/Open3D_test_data/bunny.pcd")
print(pcd)  # 输出点云点的个数
o3d.visualization.draw_geometries([pcd], window_name="原始点云",
                                  width=1024, height=768,
                                  left=50, top=50,
                                  mesh_show_back_face=False)
print("Radius oulier removal")
cl, ind = pcd.remove_radius_outlier(nb_points=16, radius=5)
radius_cloud = pcd.select_by_index(ind)
o3d.visualization.draw_geometries([radius_cloud], window_name="半径滤波",
                                  width=1024, height=768,
                                  left=50, top=50,
                                  mesh_show_back_face=False)

