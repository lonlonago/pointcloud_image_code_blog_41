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

