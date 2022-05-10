#先创建一个xyz
import numpy as np
import open3d as o3d
x = np.linspace(-3, 3, 100)          #  linspace (-3,3)之间选取均匀间隔的100个数字
mesh_x, mesh_y = np.meshgrid(x, x)   #  meshgrid 生成网格点坐标矩阵
z = np.sinc((np.power(mesh_x, 2) + np.power(mesh_y, 2)))
z_norm = (z - z.min()) / (z.max() - z.min())
xyz = np.zeros((np.size(mesh_x), 3))
xyz[:, 0] = np.reshape(mesh_x, -1)   #reshape(a,-1)自行判断生成的行数
xyz[:, 1] = np.reshape(mesh_y, -1)
xyz[:, 2] = np.reshape(z_norm, -1)
print('xyz')
print(xyz)

#%% numpy to pointcloud
# numpy保存成ply格式的点云并可视化
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(xyz)
#o3d.io.write_point_cloud("sync.ply", pcd)
o3d.visualization.draw_geometries([pcd])

#%% pointcloud to numpy
# Load saved point cloud and visualize it
pcd_load = o3d.io.read_point_cloud("/home/amadeus/PycharmProjects/Projecto3d/Open3D_test_data/bunny.pcd")

# convert Open3D.o3d.geometry.PointCloud to numpy array
xyz_load = np.asarray(pcd_load.points)
print('xyz_load')
print(xyz_load)
o3d.visualization.draw_geometries([pcd_load])

