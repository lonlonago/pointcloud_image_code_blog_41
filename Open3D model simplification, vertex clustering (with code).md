#  Vertex clustering 

##  1. Overview 

   Sometimes we want to represent a high-resolution mesh with fewer triangles, but a low-resolution mesh should still be close to a high-resolution mesh. For this Open3d implements many mesh simplification algorithms. The vertex clustering method aggregates all vertices that fall into a voxel of a given size into a single vertex. This method is implemented in simplify_vertex_clustering, where the parameter voxel_size defines the size of the voxel mesh and the contraction defines how the vertices are cluster. o3d.geometry.SimplificationContraction Average computes a simple average. 

##  2. Main functions 

  ```python  
After clicking on the GitHub Sponsor button above, you will obtain access permissions to my private code repository ( https://github.com/slowlon/my_code_bar ) to view this blog code. By searching the code number of this blog, you can find the code you need, code number is: 2024020309574444794
  ```  
##  3. Algorithm source code 

  ```python  
After clicking on the GitHub Sponsor button above, you will obtain access permissions to my private code repository ( https://github.com/slowlon/my_code_bar ) to view this blog code. By searching the code number of this blog, you can find the code you need, code number is: 2024020309574444794
  ```  
#  Code implementation 

  ```python  
After clicking on the GitHub Sponsor button above, you will obtain access permissions to my private code repository ( https://github.com/slowlon/my_code_bar ) to view this blog code. By searching the code number of this blog, you can find the code you need, code number is: 2024020309574444794
  ```  
#  III. Display of results 

 ![avatar]( 20201203220456933.png) 

 Input mesh has 35947 vertices and 69451 triangles  voxel_size = 4.865594e-03 Simplified mesh has 3222 vertices and 6454 triangles  voxel_size = 9.731187e-03 Simplified mesh has 845 vertices and 1724 triangles  

#  IV. CloudCompare Operation 

 ![avatar]( 20201229095144341.gif) 

 Mesh > Scalar field > Smooth: Smooth the scalar field associated with the mesh vertices. This method is the opposite of Gaussian Filter. This method is especially useful when using the qPCV plugin Mesh > Scalar field > Enhance: Enhance the scalar field associated with the mesh vertices. This method is especially useful when using the qPCV plugin  

