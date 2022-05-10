#DBSCAN聚类
#是一种基于密度的聚类算法
#需要两个参数：eps 定义到集群中邻居的距离，min_points 定义形成集群所需的最小点数。 该函数返回labels，其中labels -1 表示噪声
import open3d as o3d
import numpy as np
from matplotlib import pyplot as plt

ply_point_cloud = o3d.data.PLYPointCloud()
pcd = o3d.io.read_point_cloud(ply_point_cloud.path)

with o3d.utility.VerbosityContextManager(
        o3d.utility.VerbosityLevel.Debug) as cm:
    labels = np.array(
        pcd.cluster_dbscan(eps=0.02, min_points=10, print_progress=True))

max_label = labels.max()
print(f"point cloud has {max_label + 1} clusters")
colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
colors[labels < 0] = 0
pcd.colors = o3d.utility.Vector3dVector(colors[:, :3])
o3d.visualization.draw_geometries([pcd],
                                  zoom=0.455,
                                  front=[-0.4999, -0.1659, -0.8499],
                                  lookat=[2.1813, 2.0619, 2.0999],
                                  up=[0.1204, -0.9852, 0.1215])
#该算法预先计算所有点在 epsilon 半径内的所有邻居,如果选择的 epsilon 太大，这可能需要大量内存。
