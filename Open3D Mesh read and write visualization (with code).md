#  First, the main function 

 1, mesh = o3d.io read_triangle_mesh ("UV.ply") read mesh from the file, supported file formats are ply, stl, obj, off, gltf/glb 

   Compared to point cloud data structures, meshes have triangles that define a three-dimensional surface. By default, Open3D attempts to infer the file type by filename extension. The following mesh file types are supported: 

 2, mesh.vertices get vertices 3, mesh.triangles get triangular face 4, o3d.visualization.draw_geometries ([mesh]) visualization mesh 5, mesh.paint_uniform_color ([1,0.076,0]) mesh render color in RGB space [0,1] range. 6, mesh.compute_vertex_normals () computes the normal of the mesh 

#  Example code 

 1. Mesh Open3D has a data structure for 3D triangular meshes called TriangleMesh. The following code shows how to read and print its vertices and triangles from a ply file 

  ```python  
After clicking on the GitHub Sponsor button above, you will obtain access permissions to my private code repository ( https://github.com/slowlon/my_code_bar ) to view this blog code. By searching the code number of this blog, you can find the code you need, code number is: 2024020309574461801
  ```  
 The TriangleMesh class has some data fields such as vertices and triangles. Open3D provides direct memory access to these fields via numpy. 2. Visualize mesh 

  ```python  
After clicking on the GitHub Sponsor button above, you will obtain access permissions to my private code repository ( https://github.com/slowlon/my_code_bar ) to view this blog code. By searching the code number of this blog, you can find the code you need, code number is: 2024020309574461801
  ```  
 ![avatar]( 2020112017070979.png) 

 It is possible to rotate and move the mesh, but the mesh is painted a uniform gray and does not look like "3d". The reason is that the current mesh has no vertex or face normals. Therefore, use a uniform color shading instead of the more complex Phong shading. 3. Calculate the normal 

  ```python  
After clicking on the GitHub Sponsor button above, you will obtain access permissions to my private code repository ( https://github.com/slowlon/my_code_bar ) to view this blog code. By searching the code number of this blog, you can find the code you need, code number is: 2024020309574461801
  ```  
 4, mesh coloring 

  ```python  
After clicking on the GitHub Sponsor button above, you will obtain access permissions to my private code repository ( https://github.com/slowlon/my_code_bar ) to view this blog code. By searching the code number of this blog, you can find the code you need, code number is: 2024020309574461801
  ```  
#  III. Complete code 

 Realize mesh read, write, color rendering and other operations 

  ```python  
After clicking on the GitHub Sponsor button above, you will obtain access permissions to my private code repository ( https://github.com/slowlon/my_code_bar ) to view this blog code. By searching the code number of this blog, you can find the code you need, code number is: 2024020309574461801
  ```  
#  IV. Display of results 

 ![avatar]( 20201120171347820.png) 

#  V. official website 

 mesh 

