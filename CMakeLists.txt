cmake_minimum_required(VERSION 2.8)
project(304sfm_copy)

set(CMAKE_BUILD_TYPE "Release")
# 指定查找路径，例如 "C:/path/to/ExampleLib"
set(OpenCV_DIR "E:/project/opencv_contrib_4.6.0/opencv_contrib-4.6.0_build/install")


find_package(OpenCV  REQUIRED)

message("OpenCV_INCLUDE_DIRS: ${OpenCV_INCLUDE_DIRS}")
message("OpenCV_LIBS: ${OpenCV_LIBS}")
# add_executable( pose_estimation_2d2d pose_estimation_2d2d.cpp extra.cpp ) # use this if in OpenCV2 
add_executable(only_estimatePoseBoard main.cpp detect_board.cpp detect_board.h aruco_samples_ultity.hpp)
include_directories(${OpenCV_INCLUDE_DIRS})

#target_include_directories(test PRIVATE  ${OpenCV_INCLUDE_DIRS})
target_link_libraries(only_estimatePoseBoard ${OpenCV_LIBS})
