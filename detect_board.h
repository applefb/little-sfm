
#pragma once

#define _USE_MATH_DEFINES
#include <cmath>
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <chrono>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/aruco.hpp>

#include "aruco_samples_ultity.hpp"

#include "opencv2/aruco/dictionary.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/types_c.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/calib3d.hpp>
using namespace cv;


int detect_board_two(Mat camMatrix, Mat distCoeffs, Mat& transformationMatrix1, Mat& transformationMatrix2, cv::Point2f& center1, cv::Point2f& center2);



int detect_board_one(Mat camMatrix, Mat distCoeffs, std::string filename, Mat& transformationMatrix, cv::Point2f& center);


