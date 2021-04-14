#pragma once

#include "opencv2/opencv.hpp"

//负责相机标定与小球位置测算
class MyCamera
{
public:
    MyCamera();
    ~MyCamera();

    int caliImage(cv::Mat img_raw, const char* f_CamParams);
    int getBallPosition(cv::Point2d &ball_pos, cv::Point& ball_img, cv::Mat img_raw, double XAng, double YAng);
    int drawPoint(cv::Mat img_raw, cv::Mat &img_disp, cv::Point p_img, cv::Scalar color = cv::Scalar(127),int radius = 2,int thickness = 3);
    int getMask(cv::Mat &mask);
    
    int saveParams(const char* FileName);
    int readParams(const char* FileName);

private:
    cv::Size imageSize;
    cv::Point center;
    int radius;
    cv::Mat mask;
    cv::Mat trans;
    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;
    cv::Mat rvec;
    cv::Mat tvec;

    cv::Mat AInv;
    cv::Mat Rot;
    cv::Mat RInv;

    int getProcArea(cv::Mat img_raw, double proc_area[3], double mindist = 100, double param1 = 220, double param2 = 130, double minradius = 250, double maxradius = 320);
    int getCaliPnt(cv::Mat img_clp, std::vector<cv::Vec3f> &circles, double mindist = 50, double param1 = 200, double param2 = 10, int minRadius = 0, int maxRadius = 20);
    int getBallPnt(cv::Mat img_clp, std::vector<cv::Vec3f> &ball_pos, double mindist = 50, double param1 = 200, double param2 = 50, int minRadius = 20, int maxRadius = 40);
    int getBallPnt2(cv::Mat img_clp, std::vector<cv::Vec3f>& ball_pos, double mindist = 50, double param1 = 200, double param2 = 50, int minRadius = 20, int maxRadius = 40);
    int getBallPnt3(cv::Mat img_clp, std::vector<cv::Vec3f> &ball_pos);
    int removeColumn(cv::Mat &matrix, unsigned int colToRemove);
    int findNearest(cv::Mat &p_img, cv::Mat &p_obj, double thres = 60);
    int getBestTrans(cv::Mat p_view, cv::Mat p_world);

};