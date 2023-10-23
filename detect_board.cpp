#include "detect_board.h"
using namespace std;
using namespace cv;

int detect_board_two(Mat camMatrix, Mat distCoeffs, cv::Mat& transformationMatrix1, cv::Mat& transformationMatrix2, cv::Point2f& center1, cv::Point2f& center2)
{

    // �ֵ��ȡ
    cv::Ptr<cv::aruco::Dictionary> dictionary_6x6 =
        cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);

    cv::Ptr<cv::aruco::Dictionary> dictionary_4x4 =
        cv::aruco::getPredefinedDictionary(cv::aruco::DICT_ARUCO_ORIGINAL);

    // ��ȡͼƬ
    cv::Mat image1, imageCopy1;
    image1 = cv::imread("C:/Users/zhang/Pictures/apple14/IMG_0732.JPG");
    image1.copyTo(imageCopy1);


    // ��ȡͼƬ
    cv::Mat image2, imageCopy2;
    image2 = cv::imread("C:/Users/zhang/Pictures/apple14/IMG_0734.JPG");
    image2.copyTo(imageCopy2);


    // board����ָ�룬�ں�����create������ʵ�ʴ���
    // ������Щ������Ҫ�����������λ��
    cv::Ptr<cv::aruco::GridBoard> board =
        cv::aruco::GridBoard::create(5,             //ÿ�ж��ٸ�Marker
            7,             //ÿ�ж��ٸ�Marker
            0.034,          //marker����
            0.0075,          //marker֮��ļ��
            dictionary_6x6);   //�ֵ�

    std::vector<int> ids1;
    std::vector<std::vector<cv::Point2f> > corners1;
    std::vector<int> ids2;
    std::vector<std::vector<cv::Point2f> > corners2;


    // ���Marker��
    cv::aruco::detectMarkers(image1, dictionary_6x6, corners1, ids1);
    cv::aruco::detectMarkers(image2, dictionary_6x6, corners2, ids2);

    std::vector<int> ids_mark1;
    std::vector<std::vector<cv::Point2f> > corners_mark1;
    std::vector<int> ids_mark2;
    std::vector<std::vector<cv::Point2f> > corners_mark2;
    // ���Marker
    cv::aruco::detectMarkers(image1, dictionary_4x4, corners_mark1, ids_mark1);
    // ���Marker
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


    // ����һ��Scalar���󣬱�ʾBGR��ɫ����ɫͨ��Ϊ0����ɫͨ��Ϊ0����ɫͨ��Ϊ255
    cv::Scalar blueColor(255, 255, 255);

    circle(imageCopy1, center1, 10, blueColor);
    circle(imageCopy2, center2, 10, blueColor);

    // ��ʾ��ȷ��Marker
    if (ids1.size() > 0)
    {
        cout << "track thing" << endl;
        // ���Ƽ��߿�
        cv::aruco::drawDetectedMarkers(imageCopy1, corners1, ids1);
        cv::aruco::drawDetectedMarkers(imageCopy2, corners2, ids2);

        cv::aruco::drawDetectedMarkers(imageCopy1, corners_mark1, ids_mark1);
        cv::aruco::drawDetectedMarkers(imageCopy2, corners_mark2, ids_mark2);

        //        // �������λ��(�����ÿһ��marker)

        std::vector<cv::Vec3d> rvecs_mark, tvecs_mark;
        cv::aruco::estimatePoseSingleMarkers(corners_mark1, 0.044, camMatrix, distCoeffs, rvecs_mark, tvecs_mark);

        cv::drawFrameAxes(imageCopy1, camMatrix, distCoeffs, rvecs_mark, tvecs_mark, 0.1);

        cout << endl << "tvecs_mark.at(0) = " << endl;
        cout << tvecs_mark.at(0) << endl << endl;

        //// �������λ��(����� aruco ��)
        cv::Mat rvec1, tvec1;
        int valid = cv::aruco::estimatePoseBoard(corners1, ids1, board, camMatrix, distCoeffs, rvec1, tvec1);
        cv::Mat rvec2, tvec2;
        int valid2 = cv::aruco::estimatePoseBoard(corners2, ids2, board, camMatrix, distCoeffs, rvec2, tvec2);
        //cout <<"rvec.size" << rvec.size() << endl;
        ////1*3
        //cout << "tvec.size" << tvec.size() << endl;
        ////1*3







        // ����ת����ת��Ϊ��ת����
        cv::Mat rotationMatrix;
        cv::Rodrigues(rvec1, rotationMatrix);

        // ����ת�����ƽ��������ϳ�ת������
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                transformationMatrix1.at<double>(i, j) = rotationMatrix.at<double>(i, j);
            }
            transformationMatrix1.at<double>(i, 3) = tvec1.at<double>(i);
        }

        // ��ӡת������
        std::cout << "r1��" << std::endl;
        std::cout << rotationMatrix << std::endl;
        std::cout << "t1��" << std::endl;
        std::cout << tvec1 << std::endl;

        std::cout << "ת������" << std::endl;
        std::cout << transformationMatrix1 << std::endl << endl;




        // ����ת����ת��Ϊ��ת����
        cv::Mat rotationMatrix2;
        cv::Rodrigues(rvec2, rotationMatrix2);

        // ����ת�����ƽ��������ϳ�ת������
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                transformationMatrix2.at<double>(i, j) = rotationMatrix2.at<double>(i, j);
            }
            transformationMatrix2.at<double>(i, 3) = tvec2.at<double>(i);
        }

        // ��ӡת������
        std::cout << "r1��" << std::endl;
        std::cout << rotationMatrix2 << std::endl << endl;
        std::cout << "t1��" << std::endl;
        std::cout << tvec2 << std::endl << endl;
        std::cout << "ת������2��" << std::endl;
        std::cout << transformationMatrix2 << std::endl << endl;




        // draw axis for each marker

            /// �õ���λ�˹����ǣ���board����ϵ���������ϵ��

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

    // �ֵ��ȡ
    cv::Ptr<cv::aruco::Dictionary> dictionary_6x6 =
        cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);

    cv::Ptr<cv::aruco::Dictionary> dictionary_4x4 =
        cv::aruco::getPredefinedDictionary(cv::aruco::DICT_ARUCO_ORIGINAL);

    // ��ȡͼƬ
    cv::Mat image, imageCopy;
    image = cv::imread(filename);
    image.copyTo(imageCopy);



    // board����ָ�룬�ں�����create������ʵ�ʴ���
    // ������Щ������Ҫ�����������λ��
    cv::Ptr<cv::aruco::GridBoard> board =
        cv::aruco::GridBoard::create(5,             //ÿ�ж��ٸ�Marker
            7,             //ÿ�ж��ٸ�Marker
            0.034,          //marker����
            0.0075,          //marker֮��ļ��
            dictionary_6x6);   //�ֵ�

    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f> > corners;



    // ���Marker��
    cv::aruco::detectMarkers(image, dictionary_6x6, corners, ids);


    std::vector<int> ids_mark;
    std::vector<std::vector<cv::Point2f> > corners_mark;

    // ���Marker
    cv::aruco::detectMarkers(image, dictionary_4x4, corners_mark, ids_mark);


    //cout << "corners_mark1.size() = " << corners_mark.size() << endl << endl;

    //cout << "corners_mark1.at(0).size() = " << corners_mark.at(0).size() << endl << endl;


    for (const cv::Point2f& point : corners_mark.at(0)) {
        center.x += point.x;
        center.y += point.y;
    }

    center.x /= 4.0;
    center.y /= 4.0;




    // ����һ��Scalar���󣬱�ʾBGR��ɫ����ɫͨ��Ϊ0����ɫͨ��Ϊ0����ɫͨ��Ϊ255
    cv::Scalar blueColor(255, 255, 255);

    circle(imageCopy, center, 10, blueColor);


    // ��ʾ��ȷ��Marker
    if (ids.size() > 0)
    {

        // ���Ƽ��߿�
        cv::aruco::drawDetectedMarkers(imageCopy, corners, ids);
        cv::aruco::drawDetectedMarkers(imageCopy, corners_mark, ids_mark);


        //     // �������λ��(�����ÿһ��marker)

        std::vector<cv::Vec3d> rvecs_mark, tvecs_mark;
        cv::aruco::estimatePoseSingleMarkers(corners_mark, 0.044, camMatrix, distCoeffs, rvecs_mark, tvecs_mark);

        cv::drawFrameAxes(imageCopy, camMatrix, distCoeffs, rvecs_mark, tvecs_mark, 0.1);

        cout << endl << "tvecs_mark.at(0) = " << endl;
        cout << tvecs_mark.at(0) << endl << endl;

        //// �������λ��(����� aruco ��)
        cv::Mat rvec, tvec;
        int valid = cv::aruco::estimatePoseBoard(corners, ids, board, camMatrix, distCoeffs, rvec, tvec);

        //cout <<"rvec.size" << rvec.size() << endl;
        ////1*3
        //cout << "tvec.size" << tvec.size() << endl;
        ////1*3







        // ����ת����ת��Ϊ��ת����
        cv::Mat rotationMatrix;
        cv::Rodrigues(rvec, rotationMatrix);

        // ����ת�����ƽ��������ϳ�ת������
        for (int i = 0; i < 3; i++) 
        {
            for (int j = 0; j < 3; j++) 
            {
                transformationMatrix.at<double>(i, j) = rotationMatrix.at<double>(i, j);
            }
            transformationMatrix.at<double>(i, 3) = tvec.at<double>(i);
        }

        // ��ӡת������
        std::cout << "һ��R��" << std::endl;
        std::cout << rotationMatrix << std::endl;
        std::cout << "һ��T��" << std::endl;
        std::cout << tvec << std::endl;

        std::cout << "ת������" << std::endl;
        std::cout << transformationMatrix << std::endl << endl;

        std::cout << "��ɹ���һ��" << std::endl << endl;



        // draw axis for each marker

            /// �õ���λ�˹����ǣ���board����ϵ���������ϵ��

        cv::drawFrameAxes(imageCopy, camMatrix, distCoeffs, rvec, tvec, 0.1);


    }

    long long times = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

    string save_name = to_string(times);

    //cv::imwrite(save_name+".jpg", imageCopy);


    cv::waitKey(0);

    return 0;
}