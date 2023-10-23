//
//#define _USE_MATH_DEFINES
//#include <cmath>
//#include <iostream>
//#include <Eigen/Core>
//#include <Eigen/Dense>
//#include <chrono>
//#include <opencv2/aruco/charuco.hpp>
//#include <opencv2/aruco.hpp>
//
//
//#include "opencv2/viz.hpp"
//#include "opencv2/aruco/dictionary.hpp"
//#include <opencv2/core/core.hpp>
//#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/imgproc/types_c.h>
//#include <opencv2/highgui/highgui.hpp>
//#include <opencv2/core/eigen.hpp>
//#include <opencv2/calib3d.hpp>
//
//
//#include "detect_board.h"
//#include "aruco_samples_ultity.hpp"
//
//#include <Eigen/Core>
//#include <g2o/core/base_vertex.h>
//#include <g2o/core/base_unary_edge.h>
//#include <g2o/core/sparse_optimizer.h>
//#include <g2o/core/block_solver.h>
//#include <g2o/core/solver.h>
//#include <g2o/core/optimization_algorithm_gauss_newton.h>
//#include <g2o/solvers/dense/linear_solver_dense.h>
//#include <sophus/se3.hpp>
//
//
//using namespace std;
//using namespace cv;
//
//// ��������ת�����һ������
//Point2f pixel2cam(const Point2d& p, const Mat& K);
//
//typedef vector<Eigen::Vector2d> VecVector2d;
//typedef vector<Eigen::Vector3d> VecVector3d;
//
//typedef vector<Sophus::SE3d> VecSE3d;
//
//
//
//void bundleAdjustmentG2O(
//    const VecSE3d& T,
//    const VecVector2d& points_2d,
//    const Mat& K,
//    Eigen::Vector3d& XYZ);
//
//int main()
//{
//
//    //ʱ���
//    long long times = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
//    std::cout << "long" << times << endl;
//
//
//    /// <summary>
//    /// ����ڲζ�ȡ
//    /// </summary>
//    /// <returns></returns>
//    string camer_yml = "D:\\calibration\\out_apple142.yml";
//    string photo_1 = "C:/Users/zhang/Pictures/apple14/IMG_0732.JPG";
//    string photo_2 = "C:/Users/zhang/Pictures/apple14/IMG_0734.JPG";
//    string photo_3 = "C:/Users/zhang/Pictures/apple14/IMG_0733.JPG";
//    string photo_4 = "C:/Users/zhang/Pictures/apple14/IMG_0737.JPG";
//    string photo_5 = "C:/Users/zhang/Pictures/apple14/IMG_0738.JPG";
//    string photo_6 = "C:/Users/zhang/Pictures/apple14/IMG_0739.JPG";
//    string photo_7 = "C:/Users/zhang/Pictures/apple14/IMG_0740.JPG";
//    string photo_8 = "C:/Users/zhang/Pictures/apple14/IMG_0741.JPG";
//
//
//    Mat camMatrix, distCoeffs;
//    bool readOk = readCameraParameters(camer_yml, camMatrix, distCoeffs);
//    cout << "cammatrix" << endl << camMatrix << endl << distCoeffs << endl;
//
//
//    std::vector<cv::Mat> transformationMatrix_Vector;
//    for (int i = 0; i < 8; ++i) 
//    {
//        transformationMatrix_Vector.push_back(cv::Mat::eye(4, 4, CV_64F));
//    }
//
//
//    std::vector<cv::Point2f> center_vector;
//
//    for (int i = 0; i < 8; ++i)
//    {
//        center_vector.push_back(cv::Point2f(0, 0));
//    }
//
//
//
//    //detect_board_two(camMatrix, distCoeffs, transformationMatrix1, transformationMatrix2, center1, center2);
//
//
//    detect_board_one(camMatrix, distCoeffs, photo_1, transformationMatrix_Vector.at(0), center_vector.at(0));
//    detect_board_one(camMatrix, distCoeffs, photo_2, transformationMatrix_Vector.at(1), center_vector.at(1));
//    detect_board_one(camMatrix, distCoeffs, photo_3, transformationMatrix_Vector.at(2), center_vector.at(2));
//    detect_board_one(camMatrix, distCoeffs, photo_4, transformationMatrix_Vector.at(3), center_vector.at(3));
//    detect_board_one(camMatrix, distCoeffs, photo_5, transformationMatrix_Vector.at(4), center_vector.at(4));
//    detect_board_one(camMatrix, distCoeffs, photo_6, transformationMatrix_Vector.at(5), center_vector.at(5));
//    detect_board_one(camMatrix, distCoeffs, photo_7, transformationMatrix_Vector.at(6), center_vector.at(6));
//    detect_board_one(camMatrix, distCoeffs, photo_8, transformationMatrix_Vector.at(7), center_vector.at(7));
//
//
//
//
//
//    std::vector<Eigen::Vector2d> eigen_center_vector;
//    //for (const cv::Point2f& point : center_vector) {
//    //    Eigen::Vector2d eigen_point(point.x, point.y);
//    //    eigen_center_vector.push_back(eigen_point);
//    //}
//
//    for (int i = 0; i < 8; i++) 
//    {
//        if (i == 2)continue;
//        Eigen::Vector2d eigen_point(center_vector.at(i).x, center_vector.at(i).y);
//        eigen_center_vector.push_back(eigen_point);
//    }
//
//
//    std::vector<Sophus::SE3d> SE3d_vector;
//
//    for (int i = 0; i < 8; i++)
//    {
//        if (i == 2)continue;
//        Eigen::Matrix4d eigenMatrix; // ����һ��Eigen����
//        cv::cv2eigen(transformationMatrix_Vector.at(i), eigenMatrix); // ��cv::Matת��ΪEigen����
//
//        // ��ȡ��ת���󲿷֣�ǰ3x3���֣�
//        Eigen::Matrix3d rotationMatrix = eigenMatrix.block<3, 3>(0, 0);
//
//        // ��ȡƽ���������֣�ǰ3��Ԫ�ص����һ�У�
//        Eigen::Vector3d translationVector = eigenMatrix.block<3, 1>(0, 3);
//
//        // ʹ����ȡ����ת�����ƽ��������ʼ��Sophus::SE3d
//
//        SE3d_vector.push_back(Sophus::SE3d(rotationMatrix, translationVector));
//
//
//    }
//
//    Eigen::Vector3d result_g2o;
//
//    cout <<"SE3d_vector.size()" << SE3d_vector.size() << endl << endl;
//    cout <<"eigen_center_vector.size() <" << eigen_center_vector.size() << endl << endl;
//
//    bundleAdjustmentG2O(
//        SE3d_vector,
//        eigen_center_vector,
//        camMatrix,
//        result_g2o);
//
//
//
//    cv::Mat T_1_2; // ���B��������A����ξ���
//
//    // ����T_A�������
//    cv::Mat T_1_inv;
//    cv::invert(transformationMatrix_Vector.at(0), T_1_inv);
//
//    // �������B��������A����ξ���
//    T_1_2 = transformationMatrix_Vector.at(1) * T_1_inv;
//
//    //cout << "T1_2 = " << T_1_2 << endl << endl;
//
//
//    ////����ĵ㣬���Ե�һ�����ά��������ϵ
//    //Mat T1 = (Mat_<float>(3, 4) <<
//    //    1, 0, 0, 0,
//    //    0, 1, 0, 0,
//    //    0, 0, 1, 0);
//    //Mat T2 = (Mat_<float>(3, 4) <<
//    //    T_1_2.at<double>(0, 0), T_1_2.at<double>(0, 1), T_1_2.at<double>(0, 2), T_1_2.at<double>(0, 3),
//    //    T_1_2.at<double>(1, 0), T_1_2.at<double>(1, 1), T_1_2.at<double>(1, 2), T_1_2.at<double>(1, 3),
//    //    T_1_2.at<double>(2, 0), T_1_2.at<double>(2, 1), T_1_2.at<double>(2, 2), T_1_2.at<double>(2, 3)
//    //    );
//
//
//    //����ĵ㣬����mark��Ϊ����ϵ
//    //Mat T1 = (Mat_<float>(3, 4) <<
//    //    transformationMatrix1.at<double>(0, 0), transformationMatrix1.at<double>(0, 1), transformationMatrix1.at<double>(0, 2), transformationMatrix1.at<double>(0, 3),
//    //    transformationMatrix1.at<double>(1, 0), transformationMatrix1.at<double>(1, 1), transformationMatrix1.at<double>(1, 2), transformationMatrix1.at<double>(1, 3),
//    //    transformationMatrix1.at<double>(2, 0), transformationMatrix1.at<double>(2, 1), transformationMatrix1.at<double>(2, 2), transformationMatrix1.at<double>(2, 3)
//    //    );
//
//    //Mat T2 = (Mat_<float>(3, 4) <<
//    //    transformationMatrix2.at<double>(0, 0), transformationMatrix2.at<double>(0, 1), transformationMatrix2.at<double>(0, 2), transformationMatrix2.at<double>(0, 3),
//    //    transformationMatrix2.at<double>(1, 0), transformationMatrix2.at<double>(1, 1), transformationMatrix2.at<double>(1, 2), transformationMatrix2.at<double>(1, 3),
//    //    transformationMatrix2.at<double>(2, 0), transformationMatrix2.at<double>(2, 1), transformationMatrix2.at<double>(2, 2), transformationMatrix2.at<double>(2, 3)
//    //    );
//
//    // ����һ���еķ�Χ��ǰ���е�������0��1��2
//    cv::Range rowRange(0, 3);
//
//    // ʹ���еķ�Χ����ȡǰ��������
//    cv::Mat T1 = transformationMatrix_Vector.at(0)(rowRange, cv::Range::all());
//    cv::Mat T2 = transformationMatrix_Vector.at(1)(rowRange, cv::Range::all());
//
//
//
//    vector<Point2f> pts_1, pts_2;
//
//    // ����������ת�����������
//    pts_1.push_back(pixel2cam(center_vector.at(0), camMatrix));
//    pts_2.push_back(pixel2cam(center_vector.at(1), camMatrix));
//
//
//    Mat pts_4d;
//    cv::triangulatePoints(T1, T2, pts_1, pts_2, pts_4d);
//
//    vector<Point3d> points;
//
//    // ת���ɷ��������
//    for (int i = 0; i < pts_4d.cols; i++) {
//        Mat x = pts_4d.col(i);
//        x /= x.at<float>(3, 0); // ��һ��
//        Point3d p(
//            x.at<float>(0, 0),
//            x.at<float>(1, 0),
//            x.at<float>(2, 0)
//        );
//        points.push_back(p);
//    }
//
//
//
//    //std::cout << "ת������1��main" << std::endl;
//
//    //cout << transformationMatrix_Vector.at(0) << endl << endl;
//
//    //std::cout << "T1��main" << std::endl;
//
//    //cout << T1 << endl << endl;
//
//
//    //std::cout << "ת������2 main��" << std::endl;
//    //cout << transformationMatrix_Vector.at(1) << endl << endl;
//
//    //std::cout << "center1��main" << std::endl;
//
//    //cout << center_vector.at(0) << endl << endl;
//
//    //std::cout << "center2 main��" << std::endl;
//    //cout << center_vector.at(1) << endl << endl;
//
//    std::cout << "�ֶ����ǻ��Ľ��RESULTRESULTRESULTRESULTRESULTRESULTRESULT��" << std::endl;
//    cout << points.at(0) << endl;
//
//
//
//    Affine3d cam_1_pose(transformationMatrix_Vector.at(0)); // 03����任����
//    cam_1_pose = cam_1_pose.inv();
//
//    Affine3d cam_2_pose(transformationMatrix_Vector.at(1)); // 03����任����
//    cam_2_pose = cam_2_pose.inv();
//
//    Affine3d cam_3_pose(transformationMatrix_Vector.at(2)); // 03����任����
//    cam_3_pose = cam_3_pose.inv();
//
//    Affine3d cam_4_pose(transformationMatrix_Vector.at(3)); // 03����任����
//    cam_4_pose = cam_4_pose.inv();
//    Affine3d cam_5_pose(transformationMatrix_Vector.at(4)); // 03����任����
//    cam_5_pose = cam_5_pose.inv();
//
//    Affine3d cam_6_pose(transformationMatrix_Vector.at(5)); // 03����任����
//    cam_6_pose = cam_6_pose.inv();
//
//    Affine3d cam_7_pose(transformationMatrix_Vector.at(6)); // 03����任����
//    cam_7_pose = cam_7_pose.inv();
//
//    Affine3d cam_8_pose(transformationMatrix_Vector.at(7)); // 03����任����
//    cam_8_pose = cam_8_pose.inv();
//
//    Affine3d transform = Affine3d::Identity();
//    transform.translation(points.at(0));
//
//
//    viz::Widget3D w = viz::WCoordinateSystem(0.5);
//    //w.setColor(viz::Color::red());
//    w.setRenderingProperty(viz::IMMEDIATE_RENDERING, 1);
//
//    cv::Mat image1, image2;
//    image1 = cv::imread(photo_1);
//    image2 = cv::imread(photo_2);
//
//    viz::Viz3d myWindow("Viz Demo");
//
//    viz::WCameraPosition camera_frustum(Vec2f(0.889484, 0.523599), image1); //camera_frustum/cpw_frustum
//
//    //viz::WCameraPosition camera_frustum(Matx33f(3.1, 0, 0.1, 0, 3.2, 0.2, 0, 0, 1)); //��ɫ����
//   //    viz::WCameraPosition camera_frustum(Vec2f(0.889484, 0.523599)); //camera_frustum/cpw_frustum
//    //myWindow.showWidget("Camera_frustum", camera_frustum, cam_1_pose);
//
//    myWindow.showWidget("World_coordinate", w); // ��"World_coordinate"����ɫ����Ϊ��ɫ
//    //myWindow.showWidget("cam1", viz::WCameraPosition(Vec2f(0.889484, 0.523599), image1), cam_1_pose); // ����3�����λ����������ϵ��ԭ��
//    //myWindow.showWidget("cam2", viz::WCameraPosition(Vec2f(0.889484, 0.523599), image2), cam_2_pose); // ����3�����λ����������ϵ��ԭ��
//
//    myWindow.showWidget("cam1", viz::WCameraPosition(0.25), cam_1_pose); // ����3�����λ����������ϵ��ԭ��
//    myWindow.showWidget("cam2", viz::WCameraPosition(0.25), cam_2_pose); // ����3�����λ����������ϵ��ԭ��
//    myWindow.showWidget("cam3", viz::WCameraPosition(0.25), cam_3_pose); // ����3�����λ����������ϵ��ԭ��
//    myWindow.showWidget("cam4", viz::WCameraPosition(0.25), cam_4_pose); // ����3�����λ����������ϵ��ԭ��
//    myWindow.showWidget("cam5", viz::WCameraPosition(0.25), cam_5_pose); // ����3�����λ����������ϵ��ԭ��
//    myWindow.showWidget("cam6", viz::WCameraPosition(0.25), cam_6_pose); // ����3�����λ����������ϵ��ԭ��
//    myWindow.showWidget("cam7", viz::WCameraPosition(0.25), cam_7_pose); // ����3�����λ����������ϵ��ԭ��
//    myWindow.showWidget("cam8", viz::WCameraPosition(0.25), cam_8_pose); // ����3�����λ����������ϵ��ԭ��
//    
//    myWindow.showWidget("mark", viz::WCoordinateSystem(0.5), transform); // ����3�����λ����������ϵ��ԭ��
//
//    myWindow.spin();
//
//}
//
//
//
//
//Point2f pixel2cam(const Point2d& p, const Mat& K) {
//    return Point2f
//    (
//        (p.x - K.at<double>(0, 2)) / K.at<double>(0, 0),
//        (p.y - K.at<double>(1, 2)) / K.at<double>(1, 1)
//    );
//}
//
//
//
//
//
//
//
///// vertex and edges used in g2o ba
//class VertexPose_my : public g2o::BaseVertex<3, Eigen::Vector3d> 
//{
//public:
//    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
//
//    virtual void setToOriginImpl() override 
//    {
//        _estimate << 0, 0, 0;
//    }
//
//    /// left multiplication on SE3
//    virtual void oplusImpl(const double* update) override 
//    {
//        _estimate += Eigen::Vector3d(update);
//    }
//
//    virtual bool read(istream& in) override 
//    { return true; }
//
//    virtual bool write(ostream& out) const override 
//    { return true; }
//};
//
//
//class EdgeProjection_my : public g2o::BaseUnaryEdge<2, Eigen::Vector2d, VertexPose_my> 
//{
//public:
//    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
//
//    EdgeProjection_my(const Sophus::SE3d& pos, const Eigen::Matrix3d& K) : t(pos), _K(K) {}
//
//    virtual void computeError() override 
//    {
//        const VertexPose_my* v = static_cast<VertexPose_my*> (_vertices[0]);
//        Eigen::Vector3d x_estimate = v->estimate();
//        Eigen::Vector3d pos_pixel = _K * (t * x_estimate);
//        pos_pixel /= pos_pixel[2];
//        _error = _measurement - pos_pixel.head<2>();
//    }
//
//    //virtual void linearizeOplus() override {
//    //  const VertexPose_my *v = static_cast<VertexPose_my*> (_vertices[0]);
//    //  Sophus::SE3d T = v->estimate();
//    //  Eigen::Vector3d pos_cam = T * _pos3d;
//    //  double fx = _K(0, 0);
//    //  double fy = _K(1, 1);
//    //  double cx = _K(0, 2);
//    //  double cy = _K(1, 2);
//    //  double X = pos_cam[0];
//    //  double Y = pos_cam[1];
//    //  double Z = pos_cam[2];
//    //  double Z2 = Z * Z;
//    //  _jacobianOplusXi
//    //    << -fx / Z, 0, fx * X / Z2, fx * X * Y / Z2, -fx - fx * X * X / Z2, fx * Y / Z,
//    //    0, -fy / Z, fy * Y / (Z * Z), fy + fy * Y * Y / Z2, -fy * X * Y / Z2, -fy * X / Z;
//    //}
//
//    virtual bool read(istream& in) override { return true; }
//
//    virtual bool write(ostream& out) const override { return true; }
//
//private:
//    Sophus::SE3d t;
//    Eigen::Matrix3d _K;
//};
//
//
//
//
//
//
//
//
//void bundleAdjustmentG2O(
//    const VecSE3d& T,
//    const VecVector2d& points_2d,
//    const Mat& K,
//    Eigen::Vector3d & XYZ) 
//{
//
//    // ����ͼ�Ż������趨g2o
//    typedef g2o::BlockSolver<g2o::BlockSolverTraits<3, 6>> BlockSolverType;  // pose is 6, landmark is 3
//    typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType> LinearSolverType; // �������������
//    // �ݶ��½����������Դ�GN, LM, DogLeg ��ѡ
//    auto solver = new g2o::OptimizationAlgorithmGaussNewton(
//        g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));
//    g2o::SparseOptimizer optimizer;     // ͼģ��
//    optimizer.setAlgorithm(solver);   // ���������
//    optimizer.setVerbose(true);       // �򿪵������
//
//    // vertex             //[0.294733, 0.17403, 0.000187977]��ʵֵ
//    //Eigen::Vector3d  initialEstimate = Eigen::Vector3d::UnitX(); // ��λ���� (1, 0, 0)   NO
//    //Eigen::Vector3d  initialEstimate = Eigen::Vector3d (1, 1, 1); // OK
//    Eigen::Vector3d  initialEstimate = Eigen::Vector3d(0, 0, 0); // OK
//
//    VertexPose_my* vertex_pose = new VertexPose_my(); // camera vertex_pose
//    vertex_pose->setId(0);
//    vertex_pose->setEstimate(initialEstimate);
//    optimizer.addVertex(vertex_pose);
//
//    // K
//    Eigen::Matrix3d K_eigen;
//    K_eigen <<
//        K.at<double>(0, 0), K.at<double>(0, 1), K.at<double>(0, 2),
//        K.at<double>(1, 0), K.at<double>(1, 1), K.at<double>(1, 2),
//        K.at<double>(2, 0), K.at<double>(2, 1), K.at<double>(2, 2);
//
//    // edges
//    int index = 1;
//    for (size_t i = 0; i < points_2d.size(); ++i) 
//    {
//        auto p2d = points_2d[i];
//        auto Ti = T[i];
//        EdgeProjection_my* edge = new EdgeProjection_my(Ti, K_eigen);
//        edge->setId(index);
//        edge->setVertex(0, vertex_pose);
//        edge->setMeasurement(p2d);
//        edge->setInformation(Eigen::Matrix2d::Identity());
//        optimizer.addEdge(edge);
//        index++;
//    }
//
//    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
//    optimizer.initializeOptimization();
//    optimizer.optimize(10);
//    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
//    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
//    cout << "optimization costs time: " << time_used.count() << " seconds." << endl;
//    cout << "pose estimated by g2o =\n" << vertex_pose->estimate().matrix() << endl;
//    XYZ = vertex_pose->estimate();
//}





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

// ��������ת�����һ������
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

    //ʱ���
    long long times = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    std::cout << "long" << times << endl;


    /// <summary>
    /// ����ڲζ�ȡ
    /// </summary>
    /// <returns></returns>
    string camer_yml = "D:\\calibration\\out_apple142.yml";
    string photo_1 = "C:/Users/zhang/Pictures/apple14/IMG_0732.JPG";
    string photo_2 = "C:/Users/zhang/Pictures/apple14/IMG_0734.JPG";
    string photo_3 = "C:/Users/zhang/Pictures/apple14/IMG_0733.JPG";
    string photo_4 = "C:/Users/zhang/Pictures/apple14/IMG_0737.JPG";
    string photo_5 = "C:/Users/zhang/Pictures/apple14/IMG_0738.JPG";
    string photo_6 = "C:/Users/zhang/Pictures/apple14/IMG_0739.JPG";
    string photo_7 = "C:/Users/zhang/Pictures/apple14/IMG_0740.JPG";
    string photo_8 = "C:/Users/zhang/Pictures/apple14/IMG_0741.JPG";


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
        Eigen::Matrix4d eigenMatrix; // ����һ��Eigen����
        cv::cv2eigen(transformationMatrix_Vector.at(i), eigenMatrix); // ��cv::Matת��ΪEigen����

        // ��ȡ��ת���󲿷֣�ǰ3x3���֣�
        Eigen::Matrix3d rotationMatrix = eigenMatrix.block<3, 3>(0, 0);

        // ��ȡƽ���������֣�ǰ3��Ԫ�ص����һ�У�
        Eigen::Vector3d translationVector = eigenMatrix.block<3, 1>(0, 3);

        // ʹ����ȡ����ת�����ƽ��������ʼ��Sophus::SE3d

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



    cv::Mat T_1_2; // ���B��������A����ξ���

    // ����T_A�������
    cv::Mat T_1_inv;
    cv::invert(transformationMatrix_Vector.at(0), T_1_inv);

    // �������B��������A����ξ���
    T_1_2 = transformationMatrix_Vector.at(1) * T_1_inv;

    //cout << "T1_2 = " << T_1_2 << endl << endl;


    ////����ĵ㣬���Ե�һ�����ά��������ϵ
    //Mat T1 = (Mat_<float>(3, 4) <<
    //    1, 0, 0, 0,
    //    0, 1, 0, 0,
    //    0, 0, 1, 0);
    //Mat T2 = (Mat_<float>(3, 4) <<
    //    T_1_2.at<double>(0, 0), T_1_2.at<double>(0, 1), T_1_2.at<double>(0, 2), T_1_2.at<double>(0, 3),
    //    T_1_2.at<double>(1, 0), T_1_2.at<double>(1, 1), T_1_2.at<double>(1, 2), T_1_2.at<double>(1, 3),
    //    T_1_2.at<double>(2, 0), T_1_2.at<double>(2, 1), T_1_2.at<double>(2, 2), T_1_2.at<double>(2, 3)
    //    );


    //����ĵ㣬����mark��Ϊ����ϵ
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

    // ����һ���еķ�Χ��ǰ���е�������0��1��2
    cv::Range rowRange(0, 3);

    // ʹ���еķ�Χ����ȡǰ��������
    cv::Mat T1 = transformationMatrix_Vector.at(0)(rowRange, cv::Range::all());
    cv::Mat T2 = transformationMatrix_Vector.at(1)(rowRange, cv::Range::all());



    vector<Point2f> pts_1, pts_2;

    // ����������ת�����������
    pts_1.push_back(pixel2cam(center_vector.at(0), camMatrix));
    pts_2.push_back(pixel2cam(center_vector.at(1), camMatrix));


    Mat pts_4d;
    cv::triangulatePoints(T1, T2, pts_1, pts_2, pts_4d);

    vector<Point3d> points;

    // ת���ɷ��������
    for (int i = 0; i < pts_4d.cols; i++) {
        Mat x = pts_4d.col(i);
        x /= x.at<float>(3, 0); // ��һ��
        Point3d p(
            x.at<float>(0, 0),
            x.at<float>(1, 0),
            x.at<float>(2, 0)
        );
        points.push_back(p);
    }



    //std::cout << "ת������1��main" << std::endl;

    //cout << transformationMatrix_Vector.at(0) << endl << endl;

    //std::cout << "T1��main" << std::endl;

    //cout << T1 << endl << endl;


    //std::cout << "ת������2 main��" << std::endl;
    //cout << transformationMatrix_Vector.at(1) << endl << endl;

    //std::cout << "center1��main" << std::endl;

    //cout << center_vector.at(0) << endl << endl;

    //std::cout << "center2 main��" << std::endl;
    //cout << center_vector.at(1) << endl << endl;

    std::cout << "�ֶ����ǻ��Ľ��RESULTRESULTRESULTRESULTRESULTRESULTRESULT��" << std::endl;
    cout << points.at(0) << endl;



    Affine3d cam_1_pose(transformationMatrix_Vector.at(0)); // 03����任����
    cam_1_pose = cam_1_pose.inv();

    Affine3d cam_2_pose(transformationMatrix_Vector.at(1)); // 03����任����
    cam_2_pose = cam_2_pose.inv();

    Affine3d cam_3_pose(transformationMatrix_Vector.at(2)); // 03����任����
    cam_3_pose = cam_3_pose.inv();

    Affine3d cam_4_pose(transformationMatrix_Vector.at(3)); // 03����任����
    cam_4_pose = cam_4_pose.inv();
    Affine3d cam_5_pose(transformationMatrix_Vector.at(4)); // 03����任����
    cam_5_pose = cam_5_pose.inv();

    Affine3d cam_6_pose(transformationMatrix_Vector.at(5)); // 03����任����
    cam_6_pose = cam_6_pose.inv();

    Affine3d cam_7_pose(transformationMatrix_Vector.at(6)); // 03����任����
    cam_7_pose = cam_7_pose.inv();

    Affine3d cam_8_pose(transformationMatrix_Vector.at(7)); // 03����任����
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

    //viz::WCameraPosition camera_frustum(Matx33f(3.1, 0, 0.1, 0, 3.2, 0.2, 0, 0, 1)); //��ɫ����
   //    viz::WCameraPosition camera_frustum(Vec2f(0.889484, 0.523599)); //camera_frustum/cpw_frustum
    //myWindow.showWidget("Camera_frustum", camera_frustum, cam_1_pose);

    myWindow.showWidget("World_coordinate", w); // ��"World_coordinate"����ɫ����Ϊ��ɫ
    //myWindow.showWidget("cam1", viz::WCameraPosition(Vec2f(0.889484, 0.523599), image1), cam_1_pose); // ����3�����λ����������ϵ��ԭ��
    //myWindow.showWidget("cam2", viz::WCameraPosition(Vec2f(0.889484, 0.523599), image2), cam_2_pose); // ����3�����λ����������ϵ��ԭ��

    myWindow.showWidget("cam1", viz::WCameraPosition(0.25), cam_1_pose); // ����3�����λ����������ϵ��ԭ��
    myWindow.showWidget("cam2", viz::WCameraPosition(0.25), cam_2_pose); // ����3�����λ����������ϵ��ԭ��
    myWindow.showWidget("cam3", viz::WCameraPosition(0.25), cam_3_pose); // ����3�����λ����������ϵ��ԭ��
    myWindow.showWidget("cam4", viz::WCameraPosition(0.25), cam_4_pose); // ����3�����λ����������ϵ��ԭ��
    myWindow.showWidget("cam5", viz::WCameraPosition(0.25), cam_5_pose); // ����3�����λ����������ϵ��ԭ��
    myWindow.showWidget("cam6", viz::WCameraPosition(0.25), cam_6_pose); // ����3�����λ����������ϵ��ԭ��
    myWindow.showWidget("cam7", viz::WCameraPosition(0.25), cam_7_pose); // ����3�����λ����������ϵ��ԭ��
    myWindow.showWidget("cam8", viz::WCameraPosition(0.25), cam_8_pose); // ����3�����λ����������ϵ��ԭ��

    myWindow.showWidget("mark", viz::WCoordinateSystem(0.5), transform); // ����3�����λ����������ϵ��ԭ��

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

    // ����ͼ�Ż������趨g2o
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<3, 6>> BlockSolverType;  // pose is 6, landmark is 3
    typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType> LinearSolverType; // �������������
    // �ݶ��½����������Դ�GN, LM, DogLeg ��ѡ
    auto solver = new g2o::OptimizationAlgorithmGaussNewton(
        g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));
    g2o::SparseOptimizer optimizer;     // ͼģ��
    optimizer.setAlgorithm(solver);   // ���������
    optimizer.setVerbose(true);       // �򿪵������

    // vertex             //[0.294733, 0.17403, 0.000187977]��ʵֵ
    //Eigen::Vector3d  initialEstimate = Eigen::Vector3d::UnitX(); // ��λ���� (1, 0, 0)   NO
    //Eigen::Vector3d  initialEstimate = Eigen::Vector3d (1, 1, 1); // OK
    Eigen::Vector3d  initialEstimate = Eigen::Vector3d(0, 0, 0); // OK

    ////����ͷλ�˽ڵ����
    for (size_t i = 0; i < T.size(); ++i)
    {
        auto Ti = T[i];

        // ����һ��g2o::SE3Quat����
        g2o::SE3Quat g2o_se3quat;

        // ��Sophus::SE3d��������ȡƽ����������ת����
        Eigen::Vector3d translation = Ti.translation();
        Eigen::Matrix3d rotation = Ti.rotationMatrix();

        // ʹ����ȡ����Ϣ���g2o::SE3Quat����
        g2o_se3quat.setRotation(Eigen::Quaterniond(rotation));
        g2o_se3quat.setTranslation(translation);


        g2o::VertexSE3Expmap* v = new g2o::VertexSE3Expmap();
        v->setId(i+1);

        v->setFixed(true);
        
        v->setEstimate(g2o_se3quat);
        optimizer.addVertex(v);

    }



    /// <summary>
    /// ������ڵ����
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

    ///����ͷ��������
        //����ڲ�
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

