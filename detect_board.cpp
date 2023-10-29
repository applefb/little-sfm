#include "detect_board.h"
using namespace std;
using namespace cv;

int detect_board_two(Mat camMatrix, Mat distCoeffs, cv::Mat& transformationMatrix1, cv::Mat& transformationMatrix2, cv::Point2f& center1, cv::Point2f& center2)
{

    // 字典读取
    cv::Ptr<cv::aruco::Dictionary> dictionary_6x6 =
        cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);

    cv::Ptr<cv::aruco::Dictionary> dictionary_4x4 =
        cv::aruco::getPredefinedDictionary(cv::aruco::DICT_ARUCO_ORIGINAL);

    // 读取图片
    cv::Mat image1, imageCopy1;
    image1 = cv::imread("C:/Users/zhang/Pictures/apple14/IMG_0732.JPG");
    image1.copyTo(imageCopy1);


    // 读取图片
    cv::Mat image2, imageCopy2;
    image2 = cv::imread("C:/Users/zhang/Pictures/apple14/IMG_0734.JPG");
    image2.copyTo(imageCopy2);


    // board对象指针，在后面有create函数来实际创建
    // 下面这些参数需要用来计算相机位姿
    cv::Ptr<cv::aruco::GridBoard> board =
        cv::aruco::GridBoard::create(5,             //每行多少个Marker
            7,             //每列多少个Marker
            0.034,          //marker长度
            0.0075,          //marker之间的间隔
            dictionary_6x6);   //字典

    std::vector<int> ids1;
    std::vector<std::vector<cv::Point2f> > corners1;
    std::vector<int> ids2;
    std::vector<std::vector<cv::Point2f> > corners2;


    // 检测Marker板
    cv::aruco::detectMarkers(image1, dictionary_6x6, corners1, ids1);
    cv::aruco::detectMarkers(image2, dictionary_6x6, corners2, ids2);

    std::vector<int> ids_mark1;
    std::vector<std::vector<cv::Point2f> > corners_mark1;
    std::vector<int> ids_mark2;
    std::vector<std::vector<cv::Point2f> > corners_mark2;
    // 检测Marker
    cv::aruco::detectMarkers(image1, dictionary_4x4, corners_mark1, ids_mark1);
    // 检测Marker
    cv::aruco::detectMarkers(image2, dictionary_4x4, corners_mark2, ids_mark2);

    cout << "corners_mark1.size() = " << corners_mark1.size() << endl << endl;

    cout << "corners_mark1.at(0).size() = " << corners_mark1.at(0).size() << endl << endl;


    for (const cv::Point2f& point : corners_mark1.at(0)) {
        center1.x += point.x;
        center1.y += point.y;
    }

    center1.x /= 4.0;
    center1.y /= 4.0;



    for (const cv::Point2f& point : corners_mark2.at(0)) {
        center2.x += point.x;
        center2.y += point.y;
    }

    center2.x /= 4.0;
    center2.y /= 4.0;


    // 创建一个Scalar对象，表示BGR颜色，红色通道为0，绿色通道为0，蓝色通道为255
    cv::Scalar blueColor(255, 255, 255);

    circle(imageCopy1, center1, 10, blueColor);
    circle(imageCopy2, center2, 10, blueColor);

    // 显示正确的Marker
    if (ids1.size() > 0)
    {
        cout << "track thing" << endl;
        // 绘制检测边框
        cv::aruco::drawDetectedMarkers(imageCopy1, corners1, ids1);
        cv::aruco::drawDetectedMarkers(imageCopy2, corners2, ids2);

        cv::aruco::drawDetectedMarkers(imageCopy1, corners_mark1, ids_mark1);
        cv::aruco::drawDetectedMarkers(imageCopy2, corners_mark2, ids_mark2);

        //        // 估计相机位姿(相对于每一个marker)

        std::vector<cv::Vec3d> rvecs_mark, tvecs_mark;
        cv::aruco::estimatePoseSingleMarkers(corners_mark1, 0.044, camMatrix, distCoeffs, rvecs_mark, tvecs_mark);

        cv::drawFrameAxes(imageCopy1, camMatrix, distCoeffs, rvecs_mark, tvecs_mark, 0.1);

        cout << endl << "tvecs_mark.at(0) = " << endl;
        cout << tvecs_mark.at(0) << endl << endl;

        //// 估计相机位姿(相对于 aruco 板)
        cv::Mat rvec1, tvec1;
        int valid = cv::aruco::estimatePoseBoard(corners1, ids1, board, camMatrix, distCoeffs, rvec1, tvec1);
        cv::Mat rvec2, tvec2;
        int valid2 = cv::aruco::estimatePoseBoard(corners2, ids2, board, camMatrix, distCoeffs, rvec2, tvec2);
        //cout <<"rvec.size" << rvec.size() << endl;
        ////1*3
        //cout << "tvec.size" << tvec.size() << endl;
        ////1*3







        // 将旋转向量转换为旋转矩阵
        cv::Mat rotationMatrix;
        cv::Rodrigues(rvec1, rotationMatrix);

        // 将旋转矩阵和平移向量组合成转换矩阵
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                transformationMatrix1.at<double>(i, j) = rotationMatrix.at<double>(i, j);
            }
            transformationMatrix1.at<double>(i, 3) = tvec1.at<double>(i);
        }

        // 打印转换矩阵
        std::cout << "r1：" << std::endl;
        std::cout << rotationMatrix << std::endl;
        std::cout << "t1：" << std::endl;
        std::cout << tvec1 << std::endl;

        std::cout << "转换矩阵：" << std::endl;
        std::cout << transformationMatrix1 << std::endl << endl;




        // 将旋转向量转换为旋转矩阵
        cv::Mat rotationMatrix2;
        cv::Rodrigues(rvec2, rotationMatrix2);

        // 将旋转矩阵和平移向量组合成转换矩阵
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                transformationMatrix2.at<double>(i, j) = rotationMatrix2.at<double>(i, j);
            }
            transformationMatrix2.at<double>(i, 3) = tvec2.at<double>(i);
        }

        // 打印转换矩阵
        std::cout << "r1：" << std::endl;
        std::cout << rotationMatrix2 << std::endl << endl;
        std::cout << "t1：" << std::endl;
        std::cout << tvec2 << std::endl << endl;
        std::cout << "转换矩阵2：" << std::endl;
        std::cout << transformationMatrix2 << std::endl << endl;




        // draw axis for each marker

            /// 得到的位姿估计是：从board坐标系到相机坐标系的

        cv::drawFrameAxes(imageCopy1, camMatrix, distCoeffs, rvec1, tvec1, 0.1);
        cv::drawFrameAxes(imageCopy2, camMatrix, distCoeffs, rvec2, tvec2, 0.1);

    }
    //cv::namedWindow("d", 0);
    //cv::imshow("d", imageCopy1);

    long long times = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

    string save_name = to_string(times);

    cv::imwrite(save_name + "d.jpg", imageCopy1);

    //cv::namedWindow("d2", 0);
    //cv::imshow("d2", imageCopy2);
    cv::imwrite(save_name + "d2.jpg", imageCopy2);

    cv::waitKey(0);

    return 0;
}

int detect_board_one(Mat camMatrix, Mat distCoeffs, std::string filename ,Mat& transformationMatrix, cv::Point2f& center)
{

    // 字典读取
    cv::Ptr<cv::aruco::Dictionary> dictionary_6x6 =
        cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);

    cv::Ptr<cv::aruco::Dictionary> dictionary_4x4 =
        cv::aruco::getPredefinedDictionary(cv::aruco::DICT_ARUCO_ORIGINAL);

    // 读取图片
    cv::Mat image, imageCopy;
    image = cv::imread(filename);
    image.copyTo(imageCopy);



    // board对象指针，在后面有create函数来实际创建
    // 下面这些参数需要用来计算相机位姿
    cv::Ptr<cv::aruco::GridBoard> board =
        cv::aruco::GridBoard::create(5,             //每行多少个Marker
            7,             //每列多少个Marker
            0.034,          //marker长度
            0.0075,          //marker之间的间隔
            dictionary_6x6);   //字典

    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f> > corners;



    // 检测Marker板
    cv::aruco::detectMarkers(image, dictionary_6x6, corners, ids);


    std::vector<int> ids_mark;
    std::vector<std::vector<cv::Point2f> > corners_mark;

    // 检测Marker
    cv::aruco::detectMarkers(image, dictionary_4x4, corners_mark, ids_mark);


    //cout << "corners_mark1.size() = " << corners_mark.size() << endl << endl;

    //cout << "corners_mark1.at(0).size() = " << corners_mark.at(0).size() << endl << endl;


    for (const cv::Point2f& point : corners_mark.at(0)) {
        center.x += point.x;
        center.y += point.y;
    }

    center.x /= 4.0;
    center.y /= 4.0;




    // 创建一个Scalar对象，表示BGR颜色，红色通道为0，绿色通道为0，蓝色通道为255
    cv::Scalar blueColor(255, 255, 255);

    circle(imageCopy, center, 10, blueColor);


    // 显示正确的Marker
    if (ids.size() > 0)
    {

        // 绘制检测边框
        cv::aruco::drawDetectedMarkers(imageCopy, corners, ids);
        cv::aruco::drawDetectedMarkers(imageCopy, corners_mark, ids_mark);


        //     // 估计相机位姿(相对于每一个marker)

        std::vector<cv::Vec3d> rvecs_mark, tvecs_mark;
        cv::aruco::estimatePoseSingleMarkers(corners_mark, 0.044, camMatrix, distCoeffs, rvecs_mark, tvecs_mark);

        cv::drawFrameAxes(imageCopy, camMatrix, distCoeffs, rvecs_mark, tvecs_mark, 0.1);

        cout << endl << "tvecs_mark.at(0) = " << endl;
        cout << tvecs_mark.at(0) << endl << endl;

        //// 估计相机位姿(相对于 aruco 板)
        cv::Mat rvec, tvec;
        int valid = cv::aruco::estimatePoseBoard(corners, ids, board, camMatrix, distCoeffs, rvec, tvec);

        //cout <<"rvec.size" << rvec.size() << endl;
        ////1*3
        //cout << "tvec.size" << tvec.size() << endl;
        ////1*3







        // 将旋转向量转换为旋转矩阵
        cv::Mat rotationMatrix;
        cv::Rodrigues(rvec, rotationMatrix);

        // 将旋转矩阵和平移向量组合成转换矩阵
        for (int i = 0; i < 3; i++) 
        {
            for (int j = 0; j < 3; j++) 
            {
                transformationMatrix.at<double>(i, j) = rotationMatrix.at<double>(i, j);
            }
            transformationMatrix.at<double>(i, 3) = tvec.at<double>(i);
        }

        // 打印转换矩阵


        std::cout << "转换矩阵：===============================" << std::endl;
        std::cout << transformationMatrix << std::endl << endl;
        std::cout << center << std::endl << endl;
        std::cout << "完成估计一个======================================" << std::endl << endl;



        // draw axis for each marker

            /// 得到的位姿估计是：从board坐标系到相机坐标系的

        cv::drawFrameAxes(imageCopy, camMatrix, distCoeffs, rvec, tvec, 0.1);


    }

    long long times = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

    string save_name = to_string(times);

    //cv::imwrite(save_name+".jpg", imageCopy);


    cv::waitKey(0);

    return 0;
}
