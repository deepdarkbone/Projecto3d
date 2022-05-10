#包围盒
#Open3D implements an AxisAlignedBoundingBox and an OrientedBoundingBox that can also be used to crop the geometry.
import open3d as o3d
import numpy as np

#%%  AABB包围盒
pcd = o3d.io.read_point_cloud(r"/home/amadeus/PycharmProjects/Projecto3d/lamppost.pcd")
print(pcd)  # 输出点云点的个数
aabb = pcd.get_axis_aligned_bounding_box()
aabb.color = (1, 0, 0)  # aabb包围盒为红色

[center_x, center_y, center_z] = aabb.get_center()
print("aabb包围盒的中心坐标为：\n", [center_x, center_y, center_z])

vertex_set = np.asarray(aabb.get_box_points())
print("aabb包围盒的顶点为：\n", vertex_set)

aabb_box_length = np.asarray(aabb.get_extent())
print("aabb包围盒的边长为：\n", aabb_box_length)

half_extent = np.asarray(aabb.get_half_extent())
print("aabb包围盒边长的一半为：\n", half_extent)

max_bound = np.asarray(aabb.get_max_bound())
print("几何坐标的最大边界为：\n", max_bound)

max_extent = np.asarray(aabb.get_max_extent())
print("aabb包围盒边长的最大范围，即X, Y和Z轴的最大值：\n", max_extent)

min_bound = np.asarray(aabb.get_min_bound())
print("几何坐标的最小边界为：\n", min_bound)

o3d.visualization.draw_geometries([pcd, aabb], window_name="AABB包围盒",
                                  width=1024, height=768,
                                  left=50, top=50,
                                  mesh_show_back_face=False)

#%%   OBB包围盒
pcd = o3d.io.read_point_cloud(r"/home/amadeus/PycharmProjects/Projecto3d/lamppost.pcd")
print(pcd)  # 输出点云点的个数
obb = pcd.get_oriented_bounding_box()
obb.color = (0, 1, 0)  # obb包围盒为绿色

[center_x, center_y, center_z] = obb.get_center()
print("obb包围盒的中心坐标为：\n", [center_x, center_y, center_z])

vertex_set = np.asarray(obb.get_box_points())
print("obb包围盒的顶点为：\n", vertex_set)

max_bound = np.asarray(obb.get_max_bound())
print("几何坐标的最大边界为：\n", max_bound)

min_bound = np.asarray(obb.get_min_bound())
print("几何坐标的最小边界为：\n", min_bound)

o3d.visualization.draw_geometries([pcd, obb], window_name="OBB包围盒",
                                  width=1024, height=768,
                                  left=50, top=50,
                                  mesh_show_back_face=False)

