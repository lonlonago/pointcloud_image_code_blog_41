#  1. Multi-vision registration 

   Multi-frame registration is the process of aligning multiple geometric shapes in global space. Typically, the input is a set of geometric shapes (which can be point clouds or RGBD images). The output is a set of rigid transformation matrices, and the transformed point clouds can be aligned in global space. Open3d provides an interface for multi-view registration through attitude diagram estimation. For specific technical details, please refer to S. Choi, Q.-Y. Zhou, and V. Koltun, Robust Reconstruction of Indoor Scenes, CVPR, 2015. 

#  2. Enter 

   The following code reads three point clouds from three files, downsamples and visualizes them, and shows that they are not misaligned. 

  ```python  
After clicking on the GitHub Sponsor button above, you will obtain access permissions to my private code repository ( https://github.com/slowlon/my_code_bar ) to view this blog code. By searching the code number of this blog, you can find the code you need, code number is: 2024020309574441328
  ```  
 ![avatar]( 542fb63cf757461abe5570bda51fb795.png) 

#  3. Posture diagram 

   A pose graph has two key foundations: nodes and edges. A node is a set of geometries associated with an pose matrix that can be transformed into global space. A set is a set of unknown variables to be optimized. PoseGraph.nodes is a list of PoseGraphNodes. The space of the postulate is the global space, and therefore, the identity matrix. Other pose matrices are initialized by accumulating transformations between adjacent nodes. Adjacent nodes usually have a large overlap area and can be registered by Point-to-plane ICP. The edges of the pose graph connect two overlapping nodes (geometries). Each edge contains a transformation matrix that aligns the source and target geometries. This paper uses Point-to-plane ICP to estimate the transformation matrix for coarse registration. In more complex cases, the coarse registration problem is generally solved by global registration. The paper [Choi 2015.] observed that paired registration is prone to errors. Even the wrong match will be larger than the correct match, so they divided the edges of the pose graph into two categories. Odometry edges connect neighborhood nodes, and they can be aligned using local registration such as ICP. Loop closure edges connect non-neighborhood nodes. This alignment is found by less reliable global registration. In Open3d, these two types of edges are determined by the uncertain parameter in the PoseGraphEdge initializer. In addition to the rotation matrix, the user can also set the information matrix for each edge. If the information matrix is set by get_information_matrix_from_point_clouds, the loss of the edges of the pose graph will approximate the RMSE of the point set corresponding to the two sets of nodes in line process weight. For details, please refer to [Choi 2015.] and the Redwood registration benchmark. The script below creates a pose graph with three nodes and three edges. Of these edges, two are odometry edges (uncertain = False) and one is loop edge closure (uncertain = True). 

  ```python  
After clicking on the GitHub Sponsor button above, you will obtain access permissions to my private code repository ( https://github.com/slowlon/my_code_bar ) to view this blog code. By searching the code number of this blog, you can find the code you need, code number is: 2024020309574441328
  ```  
  ```python  
After clicking on the GitHub Sponsor button above, you will obtain access permissions to my private code repository ( https://github.com/slowlon/my_code_bar ) to view this blog code. By searching the code number of this blog, you can find the code you need, code number is: 2024020309574441328
  ```  
   Open3d uses function global_optimization for pose map estimation, and can choose two types of optimization algorithms, namely GlobalOptimizationGaussNewton and GlobalOptimizationLevenbergMarquardt. The reason why the latter one is recommended is because it has better convergence. GlobalOptimizationConvergenceCriteria class can be used to set the maximum number of iterations and other optimization parameters. GlobalOptimizationOption set two parameters: max_correspondence_distance defines the corresponding threshold. edge_prune_threshold is the threshold for pruning the edge of the exception. reference_node is the node ID that is regarded as the global space. 

  ```python  
After clicking on the GitHub Sponsor button above, you will obtain access permissions to my private code repository ( https://github.com/slowlon/my_code_bar ) to view this blog code. By searching the code number of this blog, you can find the code you need, code number is: 2024020309574441328
  ```  
   The global optimization is performed twice on the pose graph. The first pass will optimize the pose of the original pose graph taking into account all edges and try to distinguish between false alignments between uncertain edges. These false alignments will generate small line processing weights, which will be culled on the first pass. The second pass will run without these edges, resulting in a tighter global alignment. In this example, all edges will be considered true matches, so the second pass will terminate immediately. 

#  4. Visual optimization 

   Visualize the transformation point cloud using the draw_geometries function. 

  ```python  
After clicking on the GitHub Sponsor button above, you will obtain access permissions to my private code repository ( https://github.com/slowlon/my_code_bar ) to view this blog code. By searching the code number of this blog, you can find the code you need, code number is: 2024020309574441328
  ```  
#  5. Get the merged point cloud 

   PointCloud makes it easy to use the + operator to merge two point clouds into one. After merging, use voxel_down_sample to resample. It is recommended to post-process the point clouds after merging, as this can reduce duplicate or dense points. 

  ```python  
After clicking on the GitHub Sponsor button above, you will obtain access permissions to my private code repository ( https://github.com/slowlon/my_code_bar ) to view this blog code. By searching the code number of this blog, you can find the code you need, code number is: 2024020309574441328
  ```  
>  Note: Although this tutorial shows multi-view registration of point clouds, the same processing steps can be applied to RGBD images, as shown in the Make fragments example. 

