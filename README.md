# little-sfm
通过8张包含mark板和mark的图片，g2o图优化出mark中心点的3D位置，多视图几何
本工程是在vs2019上编写的
本工程可以当作slam14讲ch7的例子


cmakelist里面我只find opencv
其实还需要包含g2o库和eigen库

我不需要在cmakelists.txt里面包含g2o库的原因是
我用了vcpkg，它会把库安装目录给到vs2019，不需要自己连接
opencv_contrib 4.6.0需要开启viz模块

我的版本是opencv_contrib 4.6.0

vcpkg安装列表如下

PS C:\Users\zhang> vcpkg list
blas:x64-windows                                  2023-03-25         
ceres:x64-windows                                 2.1.0#4             
ceres[cxsparse]:x64-windows                                         
ceres[lapack]:x64-windows                                            
ceres[suitesparse]:x64-windows                                       
eigen3:x64-windows                                3.4.0#2           
eigen3:x86-windows                                3.4.0#2            
fmt:x64-windows                                   10.0.0             
fmt:x86-windows                                   10.0.0             
g2o:x64-windows                                   2020-02-07#4     
gflags:x64-windows                                2.2.2#7            
gklib:x64-windows                                 2022-07-27#2        
glog:x64-windows                                  0.6.0#2            
lapack-reference:x64-windows                      3.11.0#1           
lapack-reference[blas-select]:x64-windows                           
lapack-reference[noblas]:x64-windows                               
lapack:x64-windows                                2022-02-22#2        
metis:x64-windows                                 2022-07-27          
openblas:x64-windows                              0.3.23#1          
sophus:x64-windows                                1.22.10            
suitesparse:x64-windows                           5.8.0#2          
vcpkg-cmake-config:x64-windows                    2022-02-06#1
vcpkg-cmake:x64-windows                           2023-05-04
vcpkg-gfortran:x64-windows                        3#3                 Metaport to install gfortran dependencies from m...

