#涂色
import open3d as o3d
import numpy as np

pcd = o3d.io.read_point_cloud(r"/home/amadeus/PycharmProjects/Projecto3d/pointCloud.ply")
pcd.paint_uniform_color([1, 0.706, 0])
o3d.visualization.draw_geometries([pcd])