# little-sfm
通过8张包含mark板和mark的图片，g2o图优化出mark中心点的3D位置，多视图几何
本工程是在vs2019上编写的


cmakelist里面我只find opencv
其实还需要包含g2o库和eigen库

我不需要在cmakelists.txt里面包含g2o库的原因是
我用了vcpkg，它会把库安装目录给到vs2019，不需要自己连接

我的版本是opencv_contrib 4.6.0
PS C:\Users\zhang> vcpkg list
blas:x64-windows                                  2023-03-25          Metapackage for packages which provide BLAS
ceres:x64-windows                                 2.1.0#4             non-linear optimization package
ceres[cxsparse]:x64-windows                                           CXSparse support for Ceres
ceres[lapack]:x64-windows                                             Use Lapack in Ceres
ceres[suitesparse]:x64-windows                                        SuiteSparse support for Ceres
eigen3:x64-windows                                3.4.0#2             C++ template library for linear algebra: matrice...
eigen3:x86-windows                                3.4.0#2             C++ template library for linear algebra: matrice...
fmt:x64-windows                                   10.0.0              Formatting library for C++. It can be used as a ...
fmt:x86-windows                                   10.0.0              Formatting library for C++. It can be used as a ...
g2o:x64-windows                                   2020-02-07#4        g2o: A General Framework for Graph Optimization
gflags:x64-windows                                2.2.2#7             A C++ library that implements commandline flags ...
gklib:x64-windows                                 2022-07-27#2        General helper libraries for KarypisLab.
glog:x64-windows                                  0.6.0#2             C++ implementation of the Google logging module
lapack-reference:x64-windows                      3.11.0#1            LAPACK - Linear Algebra PACKage
lapack-reference[blas-select]:x64-windows                             Use external optimized BLAS
lapack-reference[noblas]:x64-windows                                  Use external optimized BLAS
lapack:x64-windows                                2022-02-22#2        Metapackage for packages which provide LAPACK
metis:x64-windows                                 2022-07-27          Serial Graph Partitioning and Fill-reducing Matr...
openblas:x64-windows                              0.3.23#1            OpenBLAS is an optimized BLAS library based on G...
sophus:x64-windows                                1.22.10             Lie group library for C++
suitesparse:x64-windows                           5.8.0#2             A suite of sparse matrix algorithms. Also provid...
vcpkg-cmake-config:x64-windows                    2022-02-06#1
vcpkg-cmake:x64-windows                           2023-05-04
vcpkg-gfortran:x64-windows                        3#3                 Metaport to install gfortran dependencies from m...


