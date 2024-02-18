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

