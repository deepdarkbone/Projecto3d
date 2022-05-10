import open3d as o3d
import numpy as np

print("Testing mesh in Open3D...")
mesh = o3d.io.read_triangle_mesh("/home/amadeus/PycharmProjects/Projecto3d/Open3D_test_data/Armadillo.ply")
print(mesh)  # 打印点数和三角面数
print('Vertices:')
print(np.asarray(mesh.vertices))  # 输出每个顶点的坐标xyz
print('Triangles:')
print(np.asarray(mesh.triangles))  # 每个面的三个点的索引
# o3d.io.write_triangle_mesh("copy_of_knot.ply", mesh) # 保存mesh
print("Computing normal and rendering it.")
mesh.compute_vertex_normals()
print(np.asarray(mesh.triangle_normals))
mesh.paint_uniform_color([1, 0.7, 0])  # 给mesh渲染颜色
o3d.visualization.draw_geometries([mesh], window_name="Open3D显示mesh模型",
                                  width=1024, height=768,
                                  left=50, top=50,
                                  mesh_show_wireframe=True,   # 是否以格网线的形式显示
                                  mesh_show_back_face=False   # 是否显示面片背景
                                  )  # 显示mesh模型

