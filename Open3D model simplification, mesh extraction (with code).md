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

