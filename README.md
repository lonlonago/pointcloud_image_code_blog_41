##  Function analysis 

 Removed half of the surface by operating directly on the triangle and triangle normal data fields of the mesh. This is done through Numpy. 

##  II. Complete code 

  ```python  
After clicking on the GitHub Sponsor button above, you will obtain access permissions to my private code repository ( https://github.com/slowlon/my_code_bar ) to view this blog code. By searching the code number of this blog, you can find the code you need, code number is: 2024020309574412817
  ```  
##  III. Display of results 

 ![avatar]( 20201201083342295.png) 



--------------------------------------------------------------------------------

#  First, the principle of the algorithm 

##  1. Algorithm overview 

   The deform_as_rigid_as_possible function in Open3D, which uses a small number of constraints to deform the triangular mesh, implements the strictest possible algorithm in Sorkine and M. Alexa, As-rigid-as-possible surface modeling, Symposium on Geometry processing, 2007. To optimize the following energy function: where: represents the rotation matrix to be optimized, and represents the vertex positions before and after optimization, respectively, representing the neighborhood set of vertices. The weight represents the cotangent weight (). 

##  2. Main functions 

  ```python  
After clicking on the GitHub Sponsor button above, you will obtain access permissions to my private code repository ( https://github.com/slowlon/my_code_bar ) to view this blog code. By searching the code number of this blog, you can find the code you need, code number is: 2024020309574420387
  ```  
 Energy and smoothed_alpha achieve a smoothed version of the ARAP goal, defined as: penalizing the deviation of adjacent rotation matrices, is the tradeoff parameter of the regularization term, and is the surface area. This smoothing goal can be used in deform_as_rigid_as_possible by using the parameters energy and smoothed_alpha together. 

##  3. References 

>  [1]Sorkine and M. Alexa, As-rigid-as-possible surface modeling, Symposium on Geometry processing, 2007. 

#  Code implementation 

  ```python  
After clicking on the GitHub Sponsor button above, you will obtain access permissions to my private code repository ( https://github.com/slowlon/my_code_bar ) to view this blog code. By searching the code number of this blog, you can find the code you need, code number is: 2024020309574420387
  ```  
#  III. Display of results 

##  1. Original image 

 ![avatar]( 20210420192636318.png) 

##  2. Deformation 

 ![avatar]( 20210420192703760.png) 



--------------------------------------------------------------------------------

#  First, the principle of the algorithm 

   The Laplace operator is an important mesh filter and is defined as follows:  

   Here is the filter strength, which is a normalized weight related to the distance from neighbors. 

#  Function analysis 

  ```python  
After clicking on the GitHub Sponsor button above, you will obtain access permissions to my private code repository ( https://github.com/slowlon/my_code_bar ) to view this blog code. By searching the code number of this blog, you can find the code you need, code number is: 2024020309574477093
  ```  
#  III. Code implementation 

  ```python  
After clicking on the GitHub Sponsor button above, you will obtain access permissions to my private code repository ( https://github.com/slowlon/my_code_bar ) to view this blog code. By searching the code number of this blog, you can find the code you need, code number is: 2024020309574477093
  ```  
#  IV. Display of results 

 ![avatar]( 20201201145517353.png) 

  filter with Laplacian with 10 iterations  filter with Laplacian with 50 iterations  

#  CloudCompare 

 ![avatar]( 20201229091738185.gif) 



--------------------------------------------------------------------------------

#  First, the principle of the algorithm 

   Open3d includes many mesh filtering algorithms, the simplest of which is the mean filter, which can be used to denoise the mesh. The value of a vertex is given by the average of the adjacent vertices. 

#  Function analysis 

  ```python  
After clicking on the GitHub Sponsor button above, you will obtain access permissions to my private code repository ( https://github.com/slowlon/my_code_bar ) to view this blog code. By searching the code number of this blog, you can find the code you need, code number is: 2024020309574489775
  ```  
#  III. Code implementation 

  ```python  
After clicking on the GitHub Sponsor button above, you will obtain access permissions to my private code repository ( https://github.com/slowlon/my_code_bar ) to view this blog code. By searching the code number of this blog, you can find the code you need, code number is: 2024020309574489775
  ```  
#  IV. Display of results 

 ![avatar]( 20201201100120180.png) 

  After one filter iteration, after five iterations  



--------------------------------------------------------------------------------

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



--------------------------------------------------------------------------------

#  Function analysis 

   Both mean filtering and Laplace filtering make the triangular mesh shrink. The Taubin filter uses two Laplace filters with different parameters to prevent mesh shrinkage. The implementation interface of this filter is: filter_smooth_taubin. 

  ```python  
After clicking on the GitHub Sponsor button above, you will obtain access permissions to my private code repository ( https://github.com/slowlon/my_code_bar ) to view this blog code. By searching the code number of this blog, you can find the code you need, code number is: 2024020309574493431
  ```  
#  Code implementation 

  ```python  
After clicking on the GitHub Sponsor button above, you will obtain access permissions to my private code repository ( https://github.com/slowlon/my_code_bar ) to view this blog code. By searching the code number of this blog, you can find the code you need, code number is: 2024020309574493431
  ```  
#  III. Display of results 

 ![avatar]( 20201201150333876.png) 



--------------------------------------------------------------------------------

#  First, the principle of the algorithm 

##  1. Implementation process 

   The minimum spanning tree method is the first and most widely used normal vector adjustment algorithm. The algorithm first determines a seed point, and then traverses the minimum spanning tree (MST) of the point cloud along a relatively flat path, adjusting the normal vector of the points on the path, so that the inner product of the normal vector of the adjacent points is positive. The MST method divides the point cloud model into two parts, the adjusted point set and the unadjusted point set of the normal vector, and then selects the two points with the smallest weight between the and to propagate the normal vector. The direction of the normal vector of the point is determined by the direction of the normal vector of the normal vector to ensure that there is no sudden change in the direction of the normal vector between the two. The selection of weights determines the direction of normal vector propagation. In practice, the algorithm first extracts a sub-point to ensure that the normal vector of the point is correct, and then propagates along the direction with the smallest weight. Every time it propagates to a new point, the normal vector of the new point is modified according to the principle that the normal vector direction does not mutate and the point set sum is updated. The whole traversal process forms a tree containing all points and the sum of the weights of the edges is the smallest. The tree is called MST, so the algorithm is called MST method. Two points in the propagation direction and an edge that constitutes MST. 

##  2. References 

>  Sun Jinhu, Zhou Lishui, An Luling. Optimization algorithm for normal vector adjustment of point cloud model [J]. Chinese Journal of Image and Graphics, 2013, 18 (07): 844-851. 

##  3. Main functions 

  ```python  
After clicking on the GitHub Sponsor button above, you will obtain access permissions to my private code repository ( https://github.com/slowlon/my_code_bar ) to view this blog code. By searching the code number of this blog, you can find the code you need, code number is: 2024020309574481376
  ```  
 Use the least cost spanning tree to orient the normal vector. 

##  4. Algorithm source code 

  ```python  
After clicking on the GitHub Sponsor button above, you will obtain access permissions to my private code repository ( https://github.com/slowlon/my_code_bar ) to view this blog code. By searching the code number of this blog, you can find the code you need, code number is: 2024020309574481376
```cpp
void PointCloud::OrientNormalsConsistentTangentPlane(size_t k) {

    if (!HasNormals()) {

        utility::LogError(

                "[OrientNormalsConsistentTangentPlane] No normals in the "

                "PointCloud. Call EstimateNormals() first.");

    }

    // Create Riemannian graph (Euclidian MST + kNN)

    // Euclidian MST is subgraph of Delaunay triangulation

    std::shared_ptr<TetraMesh> delaunay_mesh;

    std::vector<size_t> pt_map;

    std::tie(delaunay_mesh, pt_map) = TetraMesh::CreateFromPointCloud(*this);

    std::vector<WeightedEdge> delaunay_graph;

    std::unordered_set<size_t> graph_edges;

    auto EdgeIndex = [&](size_t v0, size_t v1) -> size_t {

        return std::min(v0, v1) * points_.size() + std::max(v0, v1);

    };

    auto AddEdgeToDelaunayGraph = [&](size_t v0, size_t v1) {

        v0 = pt_map[v0];

        v1 = pt_map[v1];

        size_t edge = EdgeIndex(v0, v1);

        if (graph_edges.count(edge) == 0) {

            double dist = (points_[v0] - points_[v1]).squaredNorm();

            delaunay_graph.push_back(WeightedEdge(v0, v1, dist));

            graph_edges.insert(edge);

        }

    };

    for (const Eigen::Vector4i &tetra : delaunay_mesh->tetras_) {

        AddEdgeToDelaunayGraph(tetra[0], tetra[1]);

        AddEdgeToDelaunayGraph(tetra[0], tetra[2]);

        AddEdgeToDelaunayGraph(tetra[0], tetra[3]);

        AddEdgeToDelaunayGraph(tetra[1], tetra[2]);

        AddEdgeToDelaunayGraph(tetra[1], tetra[3]);

        AddEdgeToDelaunayGraph(tetra[2], tetra[3]);

    }

    std::vector<WeightedEdge> mst = Kruskal(delaunay_graph, points_.size());

    auto NormalWeight = [&](size_t v0, size_t v1) -> double {

        return 1.0 - std::abs(normals_[v0].dot(normals_[v1]));

    };

    for (auto &edge : mst) {

        edge.weight_ = NormalWeight(edge.v0_, edge.v1_);

    }

    // Add k nearest neighbors to Riemannian graph

    KDTreeFlann kdtree(*this);

    for (size_t v0 = 0; v0 < points_.size(); ++v0) {

        std::vector<int> neighbors;

        std::vector<double> dists2;

        kdtree.SearchKNN(points_[v0], int(k), neighbors, dists2);

        for (size_t vidx1 = 0; vidx1 < neighbors.size(); ++vidx1) {

            size_t v1 = size_t(neighbors[vidx1]);

            if (v0 == v1) {

                continue;

            }

            size_t edge = EdgeIndex(v0, v1);

            if (graph_edges.count(edge) == 0) {

                double weight = NormalWeight(v0, v1);

                mst.push_back(WeightedEdge(v0, v1, weight));

                graph_edges.insert(edge);

            }

        }

    }

    // extract MST from Riemannian graph

    mst = Kruskal(mst, points_.size());

    // convert list of edges to graph

    std::vector<std::unordered_set<size_t>> mst_graph(points_.size());

    for (const auto &edge : mst) {

        size_t v0 = edge.v0_;

        size_t v1 = edge.v1_;

        mst_graph[v0].insert(v1);

        mst_graph[v1].insert(v0);

    }

    // find start node for tree traversal

    // init with node that maximizes z

    double max_z = std::numeric_limits<double>::lowest();

    size_t v0 = 0;

    for (size_t vidx = 0; vidx < points_.size(); ++vidx) {

        const Eigen::Vector3d &v = points_[vidx];

        if (v(2) > max_z) {

            max_z = v(2);

            v0 = vidx;

        }

    }

    // traverse MST and orient normals consistently

    std::queue<size_t> traversal_queue;

    std::vector<bool> visited(points_.size(), false);

    traversal_queue.push(v0);

    auto TestAndOrientNormal = [&] (const Own:: Vector3d & n0,

                                   Own:: Vector3d & n1) {

        if (n0.dot(n1) < 0) {

            n1 *= -1;

        }

    };

    TestAndOrientNormal(Eigen::Vector3d(0, 0, 1), normals_[v0]);

    while (!traversal_queue.empty()) {

        v0 = traversal_queue.front();

        traversal_queue.pop();

        visited[v0] = true;

        for (size_t v1 : mst_graph[v0]) {

            if (!visited[v1]) {

                traversal_queue.push(v1);

                TestAndNormal Orient (normal_[v0], normal_[v1]);

            }

        }

    }

}

  ```  
After clicking on the GitHub Sponsor button above, you will obtain access permissions to my private code repository ( https://github.com/slowlon/my_code_bar ) to view this blog code. By searching the code number of this blog, you can find the code you need, code number is: 2024020309574481376
  ```python  
 import open3d as o3d

import numpy as np

pcd = o3d.io.read_point_cloud("data//bunny.pcd")

pcd.paint_uniform_color([1.0, 0.0, 0.0])

pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.01, max_nn=30))

orient_normals_consistent_tangent_plane (10) #Minimum Spanning Tree

Print (np.asarray (pcd.normals) [: 10 , :]) # Outputs the normal vector of the first 10 points

O3d.visualization.draw_geometries ([pcd], point_show_normal = True, window_name = "camera orientation",

                                  width=1024, height=768,

                                  left=50, top=50,

                                  mesh_show_back_face = False) #Visualize point clouds and normals

  ```  
After clicking on the GitHub Sponsor button above, you will obtain access permissions to my private code repository ( https://github.com/slowlon/my_code_bar ) to view this blog code. By searching the code number of this blog, you can find the code you need, code number is: 2024020309574481376


--------------------------------------------------------------------------------

#  I. Overview of algorithms 

##  1. Overview of the principle 

   The mesh refinement process involves dividing each triangle into smaller triangles. The simplest way is to calculate the midpoint of each side of the triangle and divide it into four smaller triangles. This is achieved through the subdivide_midpoint function. The 3D surface and area remain the same, but the number of vertices and triangles increases. number_of_iterations parameter defines the number of refinements. 

##  2. Main functions 

  ```python  
After clicking on the GitHub Sponsor button above, you will obtain access permissions to my private code repository ( https://github.com/slowlon/my_code_bar ) to view this blog code. By searching the code number of this blog, you can find the code you need, code number is: 2024020309574415780
  ```  
##  3. References 

>  [1] Cf. Charles T. Loop, “Smooth subdivision surfaces based on triangles”,1987. 

#  Code implementation 

  ```python  
After clicking on the GitHub Sponsor button above, you will obtain access permissions to my private code repository ( https://github.com/slowlon/my_code_bar ) to view this blog code. By searching the code number of this blog, you can find the code you need, code number is: 2024020309574415780
  ```  
#  III. Display of results 

 ![avatar]( 20201203212344301.png) 

#  CloudCompare 

 ![avatar]( 20201229092605493.gif) 



--------------------------------------------------------------------------------

#  I. Overview of algorithms 

   Triangles are refined using a simple midpoint algorithm. Each iteration subdivides the triangle into four triangles, with the subdivided vertices located at the midpoint of the original triangle's edge. number_of_iterations parameter defines the number of refinements. 

##  1. Main function 

  ```python  
After clicking on the GitHub Sponsor button above, you will obtain access permissions to my private code repository ( https://github.com/slowlon/my_code_bar ) to view this blog code. By searching the code number of this blog, you can find the code you need, code number is: 2024020309574438473
  ```  
 This function transforms the triangle into four triangles covering the same surface, with the new quadrilateral vertices located at the midpoint of the sides of the original triangle. 

#  Code implementation 

  ```python  
After clicking on the GitHub Sponsor button above, you will obtain access permissions to my private code repository ( https://github.com/slowlon/my_code_bar ) to view this blog code. By searching the code number of this blog, you can find the code you need, code number is: 2024020309574438473
  ```  
#  III. Display of results 

##  1. The original model 

 ![avatar]( cf678cc3f84d48dcb99976850c81122e.png) 

##  2. The processed model 

 ![avatar]( 9aecb848a9d54a69b744eb34cf760f86.png) 



--------------------------------------------------------------------------------

#  First, model sharpening 

##  1. Overview 

   The output value () of the algorithm is the sum of the input value () plus the filter enhancement factor multiplied by the input value minus adjacent values.  

##  2. Main functions 

  ```python  
After clicking on the GitHub Sponsor button above, you will obtain access permissions to my private code repository ( https://github.com/slowlon/my_code_bar ) to view this blog code. By searching the code number of this blog, you can find the code you need, code number is: 2024020309574437404
  ```  
#  Code implementation 

  ```python  
After clicking on the GitHub Sponsor button above, you will obtain access permissions to my private code repository ( https://github.com/slowlon/my_code_bar ) to view this blog code. By searching the code number of this blog, you can find the code you need, code number is: 2024020309574437404
  ```  
#  III. Display of results 

  ```python  
After clicking on the GitHub Sponsor button above, you will obtain access permissions to my private code repository ( https://github.com/slowlon/my_code_bar ) to view this blog code. By searching the code number of this blog, you can find the code you need, code number is: 2024020309574437404
  ```  
##  1. The original model 

 ![avatar]( 4583d55794824c87b21ac440b6180806.png) 

##  2. Sharpening 

 ![avatar]( c7ed5b090e6940889a6b477f71f7c882.png) 



--------------------------------------------------------------------------------

#  First, grid extraction 

   Another way to simplify meshes is to perform step-by-step mesh extraction. Select a triangle removal that minimizes the error metric. Repeat this process until the specified number of triangles is met. Open3d implements simplify_quadric_decimation interface to minimize the error square (distance from adjacent planes), and the parameter target_number_of_triangles defines the number of facets when the extraction stops. 

##  1. Main function 

  ```python  
After clicking on the GitHub Sponsor button above, you will obtain access permissions to my private code repository ( https://github.com/slowlon/my_code_bar ) to view this blog code. By searching the code number of this blog, you can find the code you need, code number is: 2024020309574441700
  ```  
 Simplify the grid function using quadratic error metric extraction. 

##  2. Algorithm source code 

  ```python  
After clicking on the GitHub Sponsor button above, you will obtain access permissions to my private code repository ( https://github.com/slowlon/my_code_bar ) to view this blog code. By searching the code number of this blog, you can find the code you need, code number is: 2024020309574441700
  ```  
#  Code implementation 

  ```python  
After clicking on the GitHub Sponsor button above, you will obtain access permissions to my private code repository ( https://github.com/slowlon/my_code_bar ) to view this blog code. By searching the code number of this blog, you can find the code you need, code number is: 2024020309574441700
  ```  
#  III. Display of results 

  ```python  
After clicking on the GitHub Sponsor button above, you will obtain access permissions to my private code repository ( https://github.com/slowlon/my_code_bar ) to view this blog code. By searching the code number of this blog, you can find the code you need, code number is: 2024020309574441700
  ```  
##  1. The original model 

 ![avatar]( cc100f5698734984a8e095d429d4fa85.png) 

##  2. One extraction 

 ![avatar]( 9db1ae82f5f24e8baea76d579737f2c7.png) 

##  3. Secondary extraction 

 ![avatar]( e37203bfb85c4eb2b53df750e118a4cd.png) 



--------------------------------------------------------------------------------

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



--------------------------------------------------------------------------------

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



--------------------------------------------------------------------------------

#  First, the principle of the algorithm 

   Book Connection: Open3D Least Squares Fitting Ball 

#  Code implementation 

  ```python  
After clicking on the GitHub Sponsor button above, you will obtain access permissions to my private code repository ( https://github.com/slowlon/my_code_bar ) to view this blog code. By searching the code number of this blog, you can find the code you need, code number is: 2024020309574416336
  ```  
#  III. Display of results 

  ```python  
After clicking on the GitHub Sponsor button above, you will obtain access permissions to my private code repository ( https://github.com/slowlon/my_code_bar ) to view this blog code. By searching the code number of this blog, you can find the code you need, code number is: 2024020309574416336
  ```  
 ![avatar]( 9e6cb41a091c4d07ac677faa66b5b112.png) 

#  IV. The official website 

 scipy.optimize.curve_fit 



--------------------------------------------------------------------------------

#  First, the principle of the algorithm 

   See: Open3D Least Squares Fitting 2D Circles (Python Detailed Procedure Edition) 

#  Code implementation 

  ```python  
After clicking on the GitHub Sponsor button above, you will obtain access permissions to my private code repository ( https://github.com/slowlon/my_code_bar ) to view this blog code. By searching the code number of this blog, you can find the code you need, code number is: 2024020309574417145
  ```  
#  III. Display of results 

  ```python  
After clicking on the GitHub Sponsor button above, you will obtain access permissions to my private code repository ( https://github.com/slowlon/my_code_bar ) to view this blog code. By searching the code number of this blog, you can find the code you need, code number is: 2024020309574417145
  ```  
 ![avatar]( fab01a55e4714268a7261d0fc0f4e66b.png) 



--------------------------------------------------------------------------------

#  First, the principle of the algorithm 

   The polynomial curve is expressed as: the polynomial coefficient is:, and the nonlinear least squares can solve the coefficient. 

#  Code implementation 

  ```python  
After clicking on the GitHub Sponsor button above, you will obtain access permissions to my private code repository ( https://github.com/slowlon/my_code_bar ) to view this blog code. By searching the code number of this blog, you can find the code you need, code number is: 2024020309574444350
  ```  
#  III. Display of results 

  ```python  
After clicking on the GitHub Sponsor button above, you will obtain access permissions to my private code repository ( https://github.com/slowlon/my_code_bar ) to view this blog code. By searching the code number of this blog, you can find the code you need, code number is: 2024020309574444350
  ```  
 ![avatar]( 11ba4ba355094f7f9f4fd4366f018701.png) 



--------------------------------------------------------------------------------

#  First, the principle of the algorithm 

##  1, normal vector estimation 

   Based on the local surface fitting method for normal vector estimation: when the sampling surface of the point cloud is smooth everywhere, the local neighborhood of any point can be well fitted by the plane; for this reason, for each scanning point in the point cloud 

##  2, normal vector orientation 

   The normal vector calculated earlier is ambiguous, that is, only the line where the normal vector is located is obtained, but the direction of the line is not determined as the final direction of the normal vector. The normal vector is redirected by the following method: Assuming that the point cloud is dense enough and the sampling plane is smooth everywhere, then the normal vector of two adjacent points will be close to parallel. 

##  3. Surface curvature 

   The covariance matrix in (2) 

     The greater the delta, the greater the fluctuation of the neighborhood. 

##  4. References 

>  [1] Wang Fei, Liu Rufei, Ren Hongwei, Chai Yongning. Multi-phase vehicle laser point cloud registration using road target features [J]. Journal of Surveying and Mapping Science and Technology, 2020, 37 (05): 496-502. [2] Xing Zhengquan, Deng Kazhong, Xue Jiqun. Initial registration of point clouds based on K-nearest neighbor search [J]. Surveying and Mapping Science, 2013, 38 (02): 93-95. [3] Li Xinchun, Yan Zhenyu, Lin Sen, Jia Di. Point cloud registration based on neighborhood feature point extraction and matching [J]. Journal of Photonics, 2020, 49 (04): 255-265. 

#  Code implementation 

 First perform downsampling filtering and then calculate the normal of the filtered point cloud. 

  ```python  
After clicking on the GitHub Sponsor button above, you will obtain access permissions to my private code repository ( https://github.com/slowlon/my_code_bar ) to view this blog code. By searching the code number of this blog, you can find the code you need, code number is: 2024020309574498901
  ```  
#  III. Code analysis 

##  1. Normal estimation 

   Press n to view the point cloud normals. Use - and + to scale the normal length. 

   estimate_normals compute the normal for each point. This function computes the main axis by applying covariance analysis to the neighbor points. This function takes an instance of the KDTreeSearchParamHybrid class as an argument. The two key parameters here are radius = 0.01 & max_nn == 30, which is used to set the search radius and the maximum number of neighborhood points. Here set the search radius to 1cm and only consider 30 points in the neighborhood to save computing time. 

##  2. Normal orientation 

   Covariance analysis produces two normal candidates in opposite directions, both of which are correct without considering the global structure. This is called the normal problem. If there is a normal, open3d will try to align the normal position with the original normal. Otherwise, open3d will choose randomly. If you need to consider the orientation, you can use the functions: orient_normals_to_align_with_direction and orient_normals_towards_camera_location. For more details, see: Open3D Normal Estimation (2) - Normal Orientation 

##  3. Output the normal vector of the specified point 

 The normals of the estimated points can be retrieved through the normals parameter of downpcd 

  ```python  
After clicking on the GitHub Sponsor button above, you will obtain access permissions to my private code repository ( https://github.com/slowlon/my_code_bar ) to view this blog code. By searching the code number of this blog, you can find the code you need, code number is: 2024020309574498901
  ```  
##  4. Output the value of the normal vector of multiple points 

 Use help (downpcd) to view other variables. Normal vectors can be converted to numpy arrays via np.asarry. 

  ```python  
After clicking on the GitHub Sponsor button above, you will obtain access permissions to my private code repository ( https://github.com/slowlon/my_code_bar ) to view this blog code. By searching the code number of this blog, you can find the code you need, code number is: 2024020309574498901
  ```  
#  IV. Display of results 

 ![avatar]( 20200825204937848.png) 

  ```python  
After clicking on the GitHub Sponsor button above, you will obtain access permissions to my private code repository ( https://github.com/slowlon/my_code_bar ) to view this blog code. By searching the code number of this blog, you can find the code you need, code number is: 2024020309574498901
  ```  
#  V. Reference links 

 [1] Open3d学习计划——3（点云） [2] C++版本的法向量估计 



--------------------------------------------------------------------------------

#  I. Overview of algorithms 

##  1. Normal orientation 

 ![avatar]( 08cf4802dfd74e00ac37e3cce1ca5493.png) 

   Open3D provides three methods for normal orientation, namely: custom orientation, orientation with camera position orientation, and minimum cost spanning tree method for normal orientation. The normals calculated by the three methods are shown in the figure below:  

##  2. Main functions 

###  2.1 Custom Orientation 

  ```python  
After clicking on the GitHub Sponsor button above, you will obtain access permissions to my private code repository ( https://github.com/slowlon/my_code_bar ) to view this blog code. By searching the code number of this blog, you can find the code you need, code number is: 2024020309574452121
  ```  
###  2.2 Camera orientation 

  ```python  
After clicking on the GitHub Sponsor button above, you will obtain access permissions to my private code repository ( https://github.com/slowlon/my_code_bar ) to view this blog code. By searching the code number of this blog, you can find the code you need, code number is: 2024020309574452121
  ```  
 Orientation with the direction of the camera position. For any point, the normal vector is and the camera position is:, to make the normal vector towards the camera position, simply:  

###  2.3 Minimum Spanning Tree 

  ```python  
After clicking on the GitHub Sponsor button above, you will obtain access permissions to my private code repository ( https://github.com/slowlon/my_code_bar ) to view this blog code. By searching the code number of this blog, you can find the code you need, code number is: 2024020309574452121
  ```  
 Use the least cost spanning tree to orient the normal vector. 

#  Code implementation 

##  1. Customize the direction 

  ```python  
After clicking on the GitHub Sponsor button above, you will obtain access permissions to my private code repository ( https://github.com/slowlon/my_code_bar ) to view this blog code. By searching the code number of this blog, you can find the code you need, code number is: 2024020309574452121
  ```  
##  2. Results display 

 ![avatar]( ff76d401b465418a92ff3bac038ecb7a.png) 

##  3. Towards the camera position 

  ```python  
After clicking on the GitHub Sponsor button above, you will obtain access permissions to my private code repository ( https://github.com/slowlon/my_code_bar ) to view this blog code. By searching the code number of this blog, you can find the code you need, code number is: 2024020309574452121
  ```  
##  4. Display of results 

 ![avatar]( 277307122d314725a58985d9b74c8461.png) 

##  4. Minimum spanning tree 

  ```python  
After clicking on the GitHub Sponsor button above, you will obtain access permissions to my private code repository ( https://github.com/slowlon/my_code_bar ) to view this blog code. By searching the code number of this blog, you can find the code you need, code number is: 2024020309574452121
  ```  
##  5. Display of results 

 ![avatar]( 6d545fbb7f774c59951a8a947ae1cd53.png) 

#  III. Related Links 

 [1] Open3D 法线估计(1)——计算点云法向量并显示 [2] PCL 计算点云法向量并显示 



--------------------------------------------------------------------------------

##  First, the main function 

 1. Facial display 2. Vertex display 3. Obj to np array 

##  Code implementation 

  ```python  
After clicking on the GitHub Sponsor button above, you will obtain access permissions to my private code repository ( https://github.com/slowlon/my_code_bar ) to view this blog code. By searching the code number of this blog, you can find the code you need, code number is: 2024020309574428000
  ```  
##  III. Display of results 

 ![avatar]( 20201208205345463.png) 

 1. Facial display  

 ![avatar]( 20201208204334563.png) 

 2. Vertex display 3. Obj to array  



--------------------------------------------------------------------------------

#  First, the principle of the algorithm 

    The function of the pass filter is to filter out the points whose values are not within the given range in the direction of the specified dimension. The implementation principle is as follows: First, specify a dimension and the range under that dimension. Secondly, traverse each point in the point cloud to determine whether the value of the point on the specified dimension is within the range. Delete the points whose values are not within the range. Finally, the traversal is completed, and the remaining points constitute the filtered point cloud. The pass filter is simple and efficient, and is suitable for operations such as eliminating background. 

#  Code implementation 

  ```python  
After clicking on the GitHub Sponsor button above, you will obtain access permissions to my private code repository ( https://github.com/slowlon/my_code_bar ) to view this blog code. By searching the code number of this blog, you can find the code you need, code number is: 2024020309574471900
  ```  
#  III. Display of results 

##  1. X-field filtering 

 ![avatar]( a51f90a6c40540c7b10ca9a6517df3fe.png) 

##  2. Y-field filtering 

 ![avatar]( 85575ee3e00746f8b954498990b8545f.png) 

##  3. Z-field filtering 

 ![avatar]( 138ef14c5f0a449c81d8b5ff9b4d050b.png) 



--------------------------------------------------------------------------------

