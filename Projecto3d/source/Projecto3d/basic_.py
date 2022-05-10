import open3d as o3d
import numpy as np

#*******************************点云的可视化***********************************

pcd = o3d.io.read_point_cloud("milk.pcd") # 读取ply或者pcd文件

print(pcd) # 点云简单信息 》》PointCloud with 113662 points.
# print(np.asarray(pcd.points)) #打印点云

# 可视化点云
o3d.visualization.draw_geometries([pcd])

#*******************************点云降采样***********************************

print("Downsample the point cloud with a voxel of 0.05")
downpcd = pcd.voxel_down_sample(voxel_size=0.05)
o3d.visualization.draw_geometries([downpcd])

#*******************************顶点法线估计***********************************
#重新计算，一般可视化操作都带有法线估计(可以不用重新计算)
print("Recompute the normal of the downsampled point cloud")
downpcd.estimate_normals(
    search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
o3d.visualization.draw_geometries([downpcd],
                                  point_show_normal=True)

#*******************************访问顶点的法线***********************************
print("Print a normal vector of the 0th point")
print(downpcd.normals[0]) #打印第0个顶点的法线

# 打印10个点的法线估计
print("Print the normal vectors of the first 10 points")
print(np.asarray(downpcd.normals)[:10, :])

#*******************************点云的裁剪***********************************
print("Load a polygon volume and use it to crop the original point cloud")
vol = o3d.visualization.read_selection_polygon_volume(
    "../test_data/Crop/cropped.json")
chair = vol.crop_point_cloud(pcd)
o3d.visualization.draw_geometries([chair])

#*******************************画点云***********************************
print("Paint chair")
chair.paint_uniform_color([1, 0.706, 0]) #给点云统一绘制颜色，RGB范围[0,1]
o3d.visualization.draw_geometries([chair])
