#ISS特征点
import open3d as o3d
import numpy as np
import time

# 这个函数对算法没什么影响，只是突出表现一下特征点
def keypoints_to_spheres(keypoints):
        spheres = o3d.geometry.TriangleMesh()
        for keypoint in keypoints.points:
         sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.01)
         sphere.translate(keypoint)
         spheres += sphere
         spheres.paint_uniform_color([1.0, 0.0, 0.0])
        return spheres

pcd = o3d.io.read_point_cloud("/home/amadeus/PycharmProjects/Projecto3d/Open3D_test_data/bunny.pcd")

tic = time.time()
# --------------------ISS关键点提取的相关参数---------------------
keypoints = o3d.geometry.keypoint.compute_iss_keypoints(pcd,
                                                        salient_radius=0.2,
                                                        non_max_radius=0.2,
                                                        gamma_21=0.8,
                                                        gamma_32=0.5)
toc = 1000 * (time.time() - tic)
print("ISS Computation took {:.0f} [ms]".format(toc))
print("Extract",keypoints)
pcd.paint_uniform_color([0.0, 1.0, 0.0])
o3d.visualization.draw_geometries([keypoints_to_spheres(keypoints), pcd],width=800,height=800)

