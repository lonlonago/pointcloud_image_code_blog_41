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

