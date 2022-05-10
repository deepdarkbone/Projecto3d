#选择一定的点生成新点云
import open3d as o3d
import numpy as np
pcd = o3d.io.read_point_cloud("/home/amadeus/PycharmProjects/Projecto3d/Open3D_test_data/bunny.pcd")
result=[]
f = open('temp.txt','r')
lines = f.readlines()
f.close()
for line in lines:
    result.append(np.array(list(map(int,line.split(',')))))
    #print(result)

def display_inlier_outlier(cloud, ind):
    inlier_cloud = cloud.select_by_index(ind)
    outlier_cloud = cloud.select_by_index(ind, invert=True) # 设置为True表示保存ind之外的点
    print("Showing outliers (red) and inliers : ")
    outlier_cloud.paint_uniform_color([0, 1, 0])
    inlier_cloud.paint_uniform_color([1, 0, 0])
    o3d.visualization.draw_geometries([inlier_cloud, outlier_cloud],width=600,height=600)


display_inlier_outlier(pcd, result)



