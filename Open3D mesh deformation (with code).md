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

