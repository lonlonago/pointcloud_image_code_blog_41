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
