#读一个asc文件里的点云，输出

import open3d as o3d
import numpy as np

pcd = o3d.io.read_point_cloud(r"/home/amadeus/PycharmProjects/Projecto3d/0.5.asc",format='xyz')
print(pcd)#输出点云点的个数
print(np.asarray(pcd.points))#输出点的三维坐标
o3d.io.write_point_cloud("copy_of_fragment.pcd", pcd)
o3d.visualization.draw_geometries([pcd])
