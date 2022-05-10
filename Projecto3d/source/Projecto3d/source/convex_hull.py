#计算凸包，并将其可视化
# 点云的凸包是包含所有点的最小凸集
import open3d as o3d

#读点云，计算法向量，泊松磁盘采样
bunny = o3d.data.BunnyMesh()
mesh = o3d.io.read_triangle_mesh(bunny.path)
pcl = mesh.sample_points_poisson_disk(number_of_points=5000)

hull, idx = pcl.compute_convex_hull()     #计算凸包
hull_ls = o3d.geometry.LineSet.create_from_triangle_mesh(hull)  #从凸包绘制LineSet
hull_ls.paint_uniform_color((1, 0, 0))
o3d.visualization.draw_geometries([pcl, hull_ls])
