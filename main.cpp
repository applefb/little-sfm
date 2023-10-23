
#define _USE_MATH_DEFINES
#include <cmath>
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <chrono>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/aruco.hpp>


#include "opencv2/viz.hpp"
#include "opencv2/aruco/dictionary.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/types_c.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/calib3d.hpp>


#include "detect_board.h"
#include "aruco_samples_ultity.hpp"

#include <Eigen/Core>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/solver.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <g2o/core/robust_kernel_impl.h>
#include <sophus/se3.hpp>


using namespace std;
using namespace cv;

// 像素坐标转相机归一化坐标
Point2f pixel2cam(const Point2d& p, const Mat& K);

typedef vector<Eigen::Vector2d> VecVector2d;
typedef vector<Eigen::Vector3d> VecVector3d;

typedef vector<Sophus::SE3d> VecSE3d;



void bundleAdjustmentG2O(
    const VecSE3d& T,
    const VecVector2d& points_2d,
    const Mat& K,
    Eigen::Vector3d& XYZ);

int main()
{

    //时间戳
    long long times = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    std::cout << "long" << times << endl;


    /// <summary>
    /// 相机内参读取
    /// </summary>
    /// <returns></returns>
    string camer_yml = "../out_apple142.yml";
    string photo_1 = "../IMG_0732.JPG";
    string photo_2 = "../IMG_0734.JPG";
    string photo_3 = "../IMG_0733.JPG";
    string photo_4 = "../IMG_0737.JPG";
    string photo_5 = "../IMG_0738.JPG";
    string photo_6 = "../IMG_0739.JPG";
    string photo_7 = "../IMG_0740.JPG";
    string photo_8 = "../IMG_0741.JPG";


    Mat camMatrix, distCoeffs;
    bool readOk = readCameraParameters(camer_yml, camMatrix, distCoeffs);
    cout << "cammatrix" << endl << camMatrix << endl << distCoeffs << endl;


    std::vector<cv::Mat> transformationMatrix_Vector;
    for (int i = 0; i < 8; ++i)
    {
        transformationMatrix_Vector.push_back(cv::Mat::eye(4, 4, CV_64F));
    }


    std::vector<cv::Point2f> center_vector;

    for (int i = 0; i < 8; ++i)
    {
        center_vector.push_back(cv::Point2f(0, 0));
    }



    //detect_board_two(camMatrix, distCoeffs, transformationMatrix1, transformationMatrix2, center1, center2);


    detect_board_one(camMatrix, distCoeffs, photo_1, transformationMatrix_Vector.at(0), center_vector.at(0));
    detect_board_one(camMatrix, distCoeffs, photo_2, transformationMatrix_Vector.at(1), center_vector.at(1));
    detect_board_one(camMatrix, distCoeffs, photo_3, transformationMatrix_Vector.at(2), center_vector.at(2));
    detect_board_one(camMatrix, distCoeffs, photo_4, transformationMatrix_Vector.at(3), center_vector.at(3));
    detect_board_one(camMatrix, distCoeffs, photo_5, transformationMatrix_Vector.at(4), center_vector.at(4));
    detect_board_one(camMatrix, distCoeffs, photo_6, transformationMatrix_Vector.at(5), center_vector.at(5));
    detect_board_one(camMatrix, distCoeffs, photo_7, transformationMatrix_Vector.at(6), center_vector.at(6));
    detect_board_one(camMatrix, distCoeffs, photo_8, transformationMatrix_Vector.at(7), center_vector.at(7));





    std::vector<Eigen::Vector2d> eigen_center_vector;
    //for (const cv::Point2f& point : center_vector) {
    //    Eigen::Vector2d eigen_point(point.x, point.y);
    //    eigen_center_vector.push_back(eigen_point);
    //}

    for (int i = 0; i < 8; i++)
    {
        //if (i == 2)continue;
        Eigen::Vector2d eigen_point(center_vector.at(i).x, center_vector.at(i).y);
        eigen_center_vector.push_back(eigen_point);
    }


    std::vector<Sophus::SE3d> SE3d_vector;

    for (int i = 0; i < 8; i++)
    {
        //if (i == 2)continue;
        Eigen::Matrix4d eigenMatrix; // 创建一个Eigen矩阵
        cv::cv2eigen(transformationMatrix_Vector.at(i), eigenMatrix); // 将cv::Mat转换为Eigen矩阵

        // 提取旋转矩阵部分（前3x3部分）
        Eigen::Matrix3d rotationMatrix = eigenMatrix.block<3, 3>(0, 0);

        // 提取平移向量部分（前3个元素的最后一列）
        Eigen::Vector3d translationVector = eigenMatrix.block<3, 1>(0, 3);

        // 使用提取的旋转矩阵和平移向量初始化Sophus::SE3d

        SE3d_vector.push_back(Sophus::SE3d(rotationMatrix, translationVector));


    }

    Eigen::Vector3d result_g2o;

    cout << "SE3d_vector.size()" << SE3d_vector.size() << endl << endl;
    cout << "eigen_center_vector.size() <" << eigen_center_vector.size() << endl << endl;

    bundleAdjustmentG2O(
        SE3d_vector,
        eigen_center_vector,
        camMatrix,
        result_g2o);



    cv::Mat T_1_2; // 相机B相对于相机A的外参矩阵

    // 计算T_A的逆矩阵
    cv::Mat T_1_inv;
    cv::invert(transformationMatrix_Vector.at(0), T_1_inv);

    // 计算相机B相对于相机A的外参矩阵
    T_1_2 = transformationMatrix_Vector.at(1) * T_1_inv;

    //cout << "T1_2 = " << T_1_2 << endl << endl;


    ////结果的点，是以第一个相机维世界坐标系
    //Mat T1 = (Mat_<float>(3, 4) <<
    //    1, 0, 0, 0,
    //    0, 1, 0, 0,
    //    0, 0, 1, 0);
    //Mat T2 = (Mat_<float>(3, 4) <<
    //    T_1_2.at<double>(0, 0), T_1_2.at<double>(0, 1), T_1_2.at<double>(0, 2), T_1_2.at<double>(0, 3),
    //    T_1_2.at<double>(1, 0), T_1_2.at<double>(1, 1), T_1_2.at<double>(1, 2), T_1_2.at<double>(1, 3),
    //    T_1_2.at<double>(2, 0), T_1_2.at<double>(2, 1), T_1_2.at<double>(2, 2), T_1_2.at<double>(2, 3)
    //    );


    //结果的点，是以mark板为坐标系
    //Mat T1 = (Mat_<float>(3, 4) <<
    //    transformationMatrix1.at<double>(0, 0), transformationMatrix1.at<double>(0, 1), transformationMatrix1.at<double>(0, 2), transformationMatrix1.at<double>(0, 3),
    //    transformationMatrix1.at<double>(1, 0), transformationMatrix1.at<double>(1, 1), transformationMatrix1.at<double>(1, 2), transformationMatrix1.at<double>(1, 3),
    //    transformationMatrix1.at<double>(2, 0), transformationMatrix1.at<double>(2, 1), transformationMatrix1.at<double>(2, 2), transformationMatrix1.at<double>(2, 3)
    //    );

    //Mat T2 = (Mat_<float>(3, 4) <<
    //    transformationMatrix2.at<double>(0, 0), transformationMatrix2.at<double>(0, 1), transformationMatrix2.at<double>(0, 2), transformationMatrix2.at<double>(0, 3),
    //    transformationMatrix2.at<double>(1, 0), transformationMatrix2.at<double>(1, 1), transformationMatrix2.at<double>(1, 2), transformationMatrix2.at<double>(1, 3),
    //    transformationMatrix2.at<double>(2, 0), transformationMatrix2.at<double>(2, 1), transformationMatrix2.at<double>(2, 2), transformationMatrix2.at<double>(2, 3)
    //    );

    // 创建一个行的范围，前三行的索引是0、1、2
    cv::Range rowRange(0, 3);

    // 使用行的范围来获取前三行数据
    cv::Mat T1 = transformationMatrix_Vector.at(0)(rowRange, cv::Range::all());
    cv::Mat T2 = transformationMatrix_Vector.at(1)(rowRange, cv::Range::all());



    vector<Point2f> pts_1, pts_2;

    // 将像素坐标转换至相机坐标
    pts_1.push_back(pixel2cam(center_vector.at(0), camMatrix));
    pts_2.push_back(pixel2cam(center_vector.at(1), camMatrix));


    Mat pts_4d;
    cv::triangulatePoints(T1, T2, pts_1, pts_2, pts_4d);

    vector<Point3d> points;

    // 转换成非齐次坐标
    for (int i = 0; i < pts_4d.cols; i++) {
        Mat x = pts_4d.col(i);
        x /= x.at<float>(3, 0); // 归一化
        Point3d p(
            x.at<float>(0, 0),
            x.at<float>(1, 0),
            x.at<float>(2, 0)
        );
        points.push_back(p);
    }



    //std::cout << "转换矩阵1：main" << std::endl;

    //cout << transformationMatrix_Vector.at(0) << endl << endl;

    //std::cout << "T1：main" << std::endl;

    //cout << T1 << endl << endl;


    //std::cout << "转换矩阵2 main：" << std::endl;
    //cout << transformationMatrix_Vector.at(1) << endl << endl;

    //std::cout << "center1：main" << std::endl;

    //cout << center_vector.at(0) << endl << endl;

    //std::cout << "center2 main：" << std::endl;
    //cout << center_vector.at(1) << endl << endl;

    std::cout << "手动三角化的结果RESULTRESULTRESULTRESULTRESULTRESULTRESULT：" << std::endl;
    cout << points.at(0) << endl;



    Affine3d cam_1_pose(transformationMatrix_Vector.at(0)); // 03相机变换矩阵
    cam_1_pose = cam_1_pose.inv();

    Affine3d cam_2_pose(transformationMatrix_Vector.at(1)); // 03相机变换矩阵
    cam_2_pose = cam_2_pose.inv();

    Affine3d cam_3_pose(transformationMatrix_Vector.at(2)); // 03相机变换矩阵
    cam_3_pose = cam_3_pose.inv();

    Affine3d cam_4_pose(transformationMatrix_Vector.at(3)); // 03相机变换矩阵
    cam_4_pose = cam_4_pose.inv();
    Affine3d cam_5_pose(transformationMatrix_Vector.at(4)); // 03相机变换矩阵
    cam_5_pose = cam_5_pose.inv();

    Affine3d cam_6_pose(transformationMatrix_Vector.at(5)); // 03相机变换矩阵
    cam_6_pose = cam_6_pose.inv();

    Affine3d cam_7_pose(transformationMatrix_Vector.at(6)); // 03相机变换矩阵
    cam_7_pose = cam_7_pose.inv();

    Affine3d cam_8_pose(transformationMatrix_Vector.at(7)); // 03相机变换矩阵
    cam_8_pose = cam_8_pose.inv();

    Affine3d transform = Affine3d::Identity();
    transform.translation(points.at(0));


    viz::Widget3D w = viz::WCoordinateSystem(0.5);
    //w.setColor(viz::Color::red());
    w.setRenderingProperty(viz::IMMEDIATE_RENDERING, 1);

    cv::Mat image1, image2;
    image1 = cv::imread(photo_1);
    image2 = cv::imread(photo_2);

    viz::Viz3d myWindow("Viz Demo");

    viz::WCameraPosition camera_frustum(Vec2f(0.889484, 0.523599), image1); //camera_frustum/cpw_frustum

    //viz::WCameraPosition camera_frustum(Matx33f(3.1, 0, 0.1, 0, 3.2, 0.2, 0, 0, 1)); //白色菱形
   //    viz::WCameraPosition camera_frustum(Vec2f(0.889484, 0.523599)); //camera_frustum/cpw_frustum
    //myWindow.showWidget("Camera_frustum", camera_frustum, cam_1_pose);

    myWindow.showWidget("World_coordinate", w); // 将"World_coordinate"的颜色更改为红色
    //myWindow.showWidget("cam1", viz::WCameraPosition(Vec2f(0.889484, 0.523599), image1), cam_1_pose); // 创建3号相机位于世界坐标系的原点
    //myWindow.showWidget("cam2", viz::WCameraPosition(Vec2f(0.889484, 0.523599), image2), cam_2_pose); // 创建3号相机位于世界坐标系的原点

    myWindow.showWidget("cam1", viz::WCameraPosition(0.25), cam_1_pose); // 创建3号相机位于世界坐标系的原点
    myWindow.showWidget("cam2", viz::WCameraPosition(0.25), cam_2_pose); // 创建3号相机位于世界坐标系的原点
    myWindow.showWidget("cam3", viz::WCameraPosition(0.25), cam_3_pose); // 创建3号相机位于世界坐标系的原点
    myWindow.showWidget("cam4", viz::WCameraPosition(0.25), cam_4_pose); // 创建3号相机位于世界坐标系的原点
    myWindow.showWidget("cam5", viz::WCameraPosition(0.25), cam_5_pose); // 创建3号相机位于世界坐标系的原点
    myWindow.showWidget("cam6", viz::WCameraPosition(0.25), cam_6_pose); // 创建3号相机位于世界坐标系的原点
    myWindow.showWidget("cam7", viz::WCameraPosition(0.25), cam_7_pose); // 创建3号相机位于世界坐标系的原点
    myWindow.showWidget("cam8", viz::WCameraPosition(0.25), cam_8_pose); // 创建3号相机位于世界坐标系的原点

    myWindow.showWidget("mark", viz::WCoordinateSystem(0.5), transform); // 创建3号相机位于世界坐标系的原点

    myWindow.spin();

}




Point2f pixel2cam(const Point2d& p, const Mat& K) {
    return Point2f
    (
        (p.x - K.at<double>(0, 2)) / K.at<double>(0, 0),
        (p.y - K.at<double>(1, 2)) / K.at<double>(1, 1)
    );
}



void bundleAdjustmentG2O(
    const VecSE3d& T,
    const VecVector2d& points_2d,
    const Mat& K,
    Eigen::Vector3d& XYZ)
{

    // 构建图优化，先设定g2o
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<3, 6>> BlockSolverType;  // pose is 6, landmark is 3
    typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType> LinearSolverType; // 线性求解器类型
    // 梯度下降方法，可以从GN, LM, DogLeg 中选
    auto solver = new g2o::OptimizationAlgorithmGaussNewton(
        g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));
    g2o::SparseOptimizer optimizer;     // 图模型
    optimizer.setAlgorithm(solver);   // 设置求解器
    optimizer.setVerbose(true);       // 打开调试输出

    // vertex             //[0.294733, 0.17403, 0.000187977]真实值
    //Eigen::Vector3d  initialEstimate = Eigen::Vector3d::UnitX(); // 单位向量 (1, 0, 0)   NO
    //Eigen::Vector3d  initialEstimate = Eigen::Vector3d (1, 1, 1); // OK
    Eigen::Vector3d  initialEstimate = Eigen::Vector3d(0, 0, 0); // OK

    ////摄像头位姿节点添加
    for (size_t i = 0; i < T.size(); ++i)
    {
        auto Ti = T[i];

        // 创建一个g2o::SE3Quat对象
        g2o::SE3Quat g2o_se3quat;

        // 从Sophus::SE3d对象中提取平移向量和旋转矩阵
        Eigen::Vector3d translation = Ti.translation();
        Eigen::Matrix3d rotation = Ti.rotationMatrix();

        // 使用提取的信息填充g2o::SE3Quat对象
        g2o_se3quat.setRotation(Eigen::Quaterniond(rotation));
        g2o_se3quat.setTranslation(translation);


        g2o::VertexSE3Expmap* v = new g2o::VertexSE3Expmap();
        v->setId(i+1);

        v->setFixed(true);
        
        v->setEstimate(g2o_se3quat);
        optimizer.addVertex(v);

    }



    /// <summary>
    /// 特征点节点添加
    /// </summary>
    /// <param name="T"></param>
    /// <param name="points_2d"></param>
    /// <param name="K"></param>
    /// <param name="XYZ"></param>
    g2o::VertexSBAPointXYZ* vertex_pose = new g2o::VertexSBAPointXYZ();
    vertex_pose->setId(0);
    vertex_pose->setMarginalized(false);
    vertex_pose->setEstimate(initialEstimate);
    optimizer.addVertex(vertex_pose);



    // K
    Eigen::Matrix3d K_eigen;
    K_eigen <<
        K.at<double>(0, 0), K.at<double>(0, 1), K.at<double>(0, 2),
        K.at<double>(1, 0), K.at<double>(1, 1), K.at<double>(1, 2),
        K.at<double>(2, 0), K.at<double>(2, 1), K.at<double>(2, 2);

    ///摄像头参数设置
        //相机内参
    g2o::CameraParameters* camera = new g2o::CameraParameters(K.at<double>(1, 1), Eigen::Vector2d(K.at<double>(0, 2), K.at<double>(1, 2)), 0);
    camera->setId(0);
    optimizer.addParameter(camera);


    // edges
    int index = 1;
    for (size_t i = 0; i < points_2d.size(); ++i)
    {
        Eigen::Vector2d p2d = points_2d[i];


        g2o::EdgeProjectXYZ2UV* edge = new g2o::EdgeProjectXYZ2UV();
        edge->setVertex(0, dynamic_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(0)));
        edge->setVertex(1, dynamic_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(i + 1)));
        edge->setMeasurement(p2d);
        edge->setInformation(Eigen::Matrix2d::Identity());
        edge->setParameterId(0, 0);
        edge->setRobustKernel(new g2o::RobustKernelHuber());

        optimizer.addEdge(edge);

        index++;
    }

    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    optimizer.initializeOptimization();
    optimizer.optimize(10);
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
    cout << "optimization costs time: " << time_used.count() << " seconds." << endl;
    cout << "pose estimated by g2o =\n" << vertex_pose->estimate().matrix() << endl << endl;
    XYZ = vertex_pose->estimate();
}

