#kd树搜索的使用
import open3d as o3d
import numpy as np
pcd = o3d.io.read_point_cloud("/home/amadeus/PycharmProjects/Projecto3d/Open3D_test_data/1.pcd")
pcd.paint_uniform_color([0.5, 0.5, 0.5])#把所有点渲染为灰色（灰兔子）
pcd_tree = o3d.geometry.KDTreeFlann(pcd)#建立KD树索引
pcd.colors[200] = [1, 0, 0]#给定查询点并渲染为红色
#---------------K近邻搜索（KNN）----------------
[k, idx, _] = pcd_tree.search_knn_vector_3d(pcd.points[200], 200)# K近邻搜索，
                                                 # 返回值int, open3d.utility.IntVector, open3d.utility.DoubleVector
                                                 #k表示point[i]的k个近邻点，idx表示point[i]的k个近邻点的索引，dis表示point[i]到近邻点的平方距离。
                                                 # 搜索到的点包含自己
np.asarray(pcd.colors)[idx[1:], :] = [0, 1, 0]#K邻域的点，渲染为绿色
#---------------半径搜索（RNN）-----------------
pcd.colors[1500] = [1, 0, 0]#给定查询点并渲染为红色
[k1, idx1, _] = pcd_tree.search_radius_vector_3d(pcd.points[1500], 0.02)#半径搜索
np.asarray(pcd.colors)[idx1[1:], :] = [0, 0, 1]#半径搜索结果并渲染为蓝色
#---------------混合搜索-----------------
pcd.colors[3000] = [1, 1, 0]#给定查询点并渲染为黄色
[k2, idx2, _] = pcd_tree.search_hybrid_vector_3d(pcd.points[3000], 0.05,200)#K近邻搜索
np.asarray(pcd.colors)[idx2[1:], :] = [0, 1, 0.8]#半径搜索结果并渲染为青色
o3d.visualization.draw_geometries([pcd])
