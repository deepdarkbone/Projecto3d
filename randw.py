import open3d as o3d

print("Testing IO for point cloud ...")
sample_pcd_data = o3d.data.PCDPointCloud()
pcd = o3d.io.read_point_cloud(sample_pcd_data.path)
print(pcd)
o3d.visualization.draw_geometries([pcd], width=800, height=600)
o3d.io.write_point_cloud("copy_of_fragment.pcd", pcd)