#include <iostream>
#include "MyCamera.h"

MyCamera::MyCamera() {};
MyCamera::~MyCamera() {};

int MyCamera::caliImage(cv::Mat img_raw, const char* f_CamParams) {

    short rtn;
    cv::Mat img_proc = img_raw.clone();
    cv::Mat img_disp;

    cv::GaussianBlur(img_proc, img_proc, cv::Size(5, 5), 10);

    //得到大致圆盘区域
    double proc_area[3];
    rtn = getProcArea(img_proc, proc_area);
    if (rtn) {
        std::cout << "Bad parameters for getProcArea!" << std::endl;
        return -1;
    }
    imageSize = img_proc.size();
    center = cv::Point(cvRound(proc_area[0]), cvRound(proc_area[1]));
    radius = cvRound(proc_area[2]);

    //截取圆盘图像
    cv::Mat img_clp;
    mask = cv::Mat::zeros(img_proc.size(), CV_8UC1);
    cv::circle(mask, center, radius, cv::Scalar(255), -1);
    img_proc.copyTo(img_clp, mask);
    //cv::imshow("debug",img_clp);
    //cv::waitKey(0);

    //图像二值化
    //cv::threshold(img_clp, img_clp, 200, 255, cv::THRESH_BINARY);

    //求取标定点坐标
    std::vector<cv::Vec3f> circles_cali;
    rtn = getCaliPnt(img_clp, circles_cali, 30, 200, 10, 0, radius / 30);
    if (rtn) {
        std::cout << "Bad parameters for getCaliPnt!" << std::endl;
        return -1;
    }

    std::vector<cv::Point2d> pnt_img_dist, pnt_img;
    for (int i = 0; i < (int)circles_cali.size();i++) {
        pnt_img_dist.push_back(cv::Point2d((double)circles_cali[i][0], (double)circles_cali[i][1]));
    }

    //求取相机参数
    readParams(f_CamParams);

    AInv = cameraMatrix.inv();
    //cv::undistortPoints(pnt_img_dist, pnt_img, cameraMatrix, distCoeffs, cv::noArray(), cameraMatrix);
    //cv::undistort(img_clp, img_disp, cameraMatrix, distCoeffs);
    //for (int i = 0; i < (int)circles_cali.size();i++) {
    //	cv::Point p_center = cv::Point(cvRound(pnt_img[i].x), cvRound(pnt_img[i].y));
    //	cv::circle(img_disp, p_center, 5, cv::Scalar(127), 3);
    //}
    //cv::imshow("Proc Image", img_disp);
    //cv::waitKey(0);

    //预估变换矩阵将实际标定点转为预估标定点在图像中的位置
    double k = radius / 140.0 / sqrt(2);

    trans = cv::Mat::eye(3, 3, CV_64FC1);
    trans.at<double>(0, 0) = k;
    trans.at<double>(0, 1) = k;
    trans.at<double>(1, 0) = k;
    trans.at<double>(1, 1) = -k;
    trans.at<double>(0, 2) = center.x;
    trans.at<double>(1, 2) = center.y;

    cv::Matx<double, 3, 9> pnt_obj;
    pnt_obj << -120, -60, 0, -60, 0, 60, 0, 60, 120,
        0, 60, 120, -60, 0, 60, -120, -60, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0;

    //匹配特征点，除去距离过大的点
    cv::Mat pnt_img_match = cv::Mat(2, pnt_img_dist.size(), CV_64FC1);
    cv::Mat pnt_obj_match(pnt_obj);
    for (int i = 0;i < (int)pnt_img_dist.size();i++) {
        pnt_img_match.at<double>(0, i) = pnt_img_dist[i].x;
        pnt_img_match.at<double>(1, i) = pnt_img_dist[i].y;
    }

    rtn = findNearest(pnt_img_match, pnt_obj_match);
    if (rtn) {
        std::cout << "findNearest false!" << std::endl;
        return -1;
    }

    if (pnt_img_match.cols < 6) {
        std::cout << "Not enough match point!" << std::endl;
        return -1;
    }
    
    cv::solvePnPRansac(pnt_obj_match.t(), pnt_img_match.t(), cameraMatrix, distCoeffs, rvec, tvec);

    cv::Rodrigues(rvec, Rot);
    RInv = Rot.inv();

    //cv::Mat pnt_tmp = cv::Mat::zeros(3, 1, CV_64FC1);
    //pnt_tmp.at<double>(0, 0) = 0;
    //cv::Mat R;
    //cv::Rodrigues(rvec, R);
    //cv::Mat pnt_tmp2 = cameraMatrix * (R * pnt_tmp + tvec);
    //double s = pnt_tmp2.at<double>(2, 0);
    //cv::Point pnt_tmp3;
    //pnt_tmp3.x = pnt_tmp2.at<double>(0, 0) / s;
    //pnt_tmp3.y = pnt_tmp2.at<double>(1, 0) / s;


    cv::Mat center_tmp;
    center_tmp = cameraMatrix * tvec;
    double s = center_tmp.at<double>(2, 0);
    center.x = int(center_tmp.at<double>(0, 0) / s);
    center.y = int(center_tmp.at<double>(1, 0) / s);
    radius -= 10;

    mask = cv::Mat::zeros(img_proc.size(), CV_8UC1);
    cv::circle(mask, center, radius, cv::Scalar(255), -1);

    cv::Mat img_debug;
    cv::undistort(img_raw, img_debug, cameraMatrix, distCoeffs);
    cv::imshow("debug", img_debug);
    cv::waitKey(0);

    return 0;
}

int MyCamera::getBallPosition(cv::Point2d &ball_pos, cv::Point &ball_img,cv::Mat img_raw, double XAng, double YAng) {

    cv::Mat img_clp;
    img_raw.copyTo(img_clp, mask);

    //cv::GaussianBlur(img_clp, img_clp, cv::Size(5, 5), 10);


    //求取小球相对圆盘坐标
    std::vector<cv::Vec3f> circle_ball;
    if (getBallPnt(img_clp, circle_ball, 100, 200, 50, radius / 10, radius / 8)) {
        std::cout << "No ball detected!" << std::endl;

        cv::String filename;
        static int index = 0;
        filename = cv::format("../BallNotFoundImg/img_%d.png", index++);
        cv::imwrite(filename, img_clp);
        return -1;
    }
    //getBallPnt3(img_clp, circle_ball);

    ball_img.x = (int)circle_ball[0].val[0];
    ball_img.y = (int)circle_ball[0].val[1];

    std::vector<cv::Point2d> ball_vec_dist, ball_vec;
    ball_vec_dist.push_back(cv::Point2d((double)circle_ball[0].val[0], (double)circle_ball[0].val[1]));
    cv::undistortPoints(ball_vec_dist, ball_vec, cameraMatrix, distCoeffs, cv::noArray(), cameraMatrix);

    double sx, sy, cx, cy;
    sx = sin(XAng);
    sy = sin(YAng);
    cx = cos(XAng);
    cy = cos(YAng);

    cv::Matx33d rotPlate;
    rotPlate << cy,     sx * sy,     sx * cy,
                0,      cy,          -sy,
                -sy,    cx * sy,     cx * cy;
    
    cv::Mat RotExInv;
    RotExInv = (Rot * rotPlate).inv();

    cv::Matx31d m_;
    m_ << ball_vec[0].x, ball_vec[0].y, 1;

    cv::Mat RIAIm_, RIt;
    RIAIm_ = RotExInv * AInv * m_;
    RIt = RotExInv * tvec;

    double s = (15 + RIt.at<double>(2, 0)) / RIAIm_.at<double>(2, 0);
    
    cv::Mat M_;
    M_ = s * RIAIm_ - RIt;

    ball_pos.x = M_.at<double>(0, 0);
    ball_pos.y = M_.at<double>(1, 0);

    return 0;
}

int MyCamera::drawPoint(cv::Mat img_raw, cv::Mat &img_disp, cv::Point p_img, cv::Scalar color, int radius, int thickness) {
    img_disp = img_raw.clone();
    
    cv::circle(img_disp, p_img, radius, color, thickness);

    return 0;
}

int MyCamera::getMask(cv::Mat& mask) {

    //cv::imshow("debug", this->mask);
    //cv::waitKey(0);
    mask = this->mask.clone();

    return 0;
}

int MyCamera::saveParams(const char* FileName)
{
    cv::FileStorage fs(FileName, cv::FileStorage::WRITE);

    time_t tt;
    time(&tt);
    struct tm* t2 = new tm();
    localtime_s(t2, &tt);
    char strTime[50];
    strftime(strTime, sizeof(strTime) - 1, "%c", t2);

    fs << "calibration_time" << strTime;
    fs << "imageSize" << imageSize;
    fs << "center" << center;
    fs << "radius" << radius;
    fs << "trans" << trans;
    fs << "cameraMatrix" << cameraMatrix;
    fs << "distCoeffs" << distCoeffs;
    fs << "rvec" << rvec;
    fs << "tvec" << tvec;

    fs.release();

    return 0;
}

int MyCamera::readParams(const char* FileName)
{
    cv::FileStorage fs(FileName, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        return -1;
    }

    fs["imageSize"] >> imageSize;
    fs["center"] >> center;
    fs["radius"] >> radius;
    fs["trans"] >> trans;
    fs["cameraMatrix"] >> cameraMatrix;
    fs["distCoeffs"] >> distCoeffs;
    fs["rvec"] >> rvec;
    fs["tvec"] >> tvec;

    mask = cv::Mat::zeros(imageSize, CV_8UC1);
    cv::circle(mask, center, radius, cv::Scalar(255), -1);
    AInv = cameraMatrix.inv();
    cv::Rodrigues(rvec, Rot);
    RInv = Rot.inv();

    fs.release();

    return 0;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int MyCamera::getProcArea(cv::Mat img_raw, double proc_area[3], double mindist, double param1, double param2, double minradius, double maxradius) {

    int k = 10;
    cv::Mat img_proc = img_raw.clone();
    cv::resize(img_proc, img_proc, cv::Size(img_proc.cols / k, img_proc.rows / k));

    std::vector<cv::Vec3f> circles;
    while (true) {
        cv::HoughCircles(img_proc, circles, cv::HOUGH_GRADIENT, 1, mindist, param1, param2 / k, (int)(minradius / k), (int)(maxradius / k));

        if (circles.size() == 1) {
            break;
        }
        else if (circles.size() == 0) {
            param2 /= 1.1;
            printf("No circle detected! param2 = %.1f\n", param2);
        }
        else {
            param2 *= 1.05;
            printf("Too many circles detected! param2 = %.1f\n", param2);
        }
        if (param2 <= 5 || param2 >= 300) {
            return -1;
        }
    }

    proc_area[0] = (double)circles[0][0] * k;
    proc_area[1] = (double)circles[0][1] * k;
    proc_area[2] = (double)circles[0][2] * k;

    return 0;
}

int MyCamera::getCaliPnt(cv::Mat img_clp, std::vector<cv::Vec3f> &circles, double mindist, double param1, double param2, int minRadius, int maxRadius) {

    int minCircles = 6;
    int maxCircles = 9;

    cv::Mat img_proc;
    cv::GaussianBlur(img_clp, img_proc, cv::Size(3, 3), 0, 0);

    int maxiter = 20;
    int times = 0;
    while (true) {
        cv::GaussianBlur(img_proc, img_proc, cv::Size(3, 3), 0, 0);
        cv::HoughCircles(img_proc, circles, cv::HOUGH_GRADIENT, 1, mindist, param1, param2, minRadius, maxRadius);

        if ((int)circles.size() >= minCircles && (int)circles.size() <= maxCircles) {
            break;
        }
        else if ((int)circles.size() < minCircles) {
            param2 /= 1.1;
            printf("Not enough point detected! param2 = %.1f\n", param2);
        }
        else if ((int)circles.size() > maxCircles) {
            param2 *= 1.05;
            printf("Too many points detected! param2 = %.1f\n", param2);
        }
        if (param2 <= 3 || param2 >= 200) {
            return -1;
        }
        times++;
        if (times > maxiter) {
            printf("getCaliPnt max iteration!\n");
            return -1;
        }
    }

    return 0;
}

int MyCamera::getBallPnt(cv::Mat img_clp, std::vector<cv::Vec3f> &ball_pos, double mindist, double param1, double param2, int minRadius, int maxRadius) {

    int k = 10;
    cv::Mat img_proc = img_clp.clone();
    cv::resize(img_proc, img_proc, cv::Size(img_clp.cols / k, img_clp.rows / k));
    cv::GaussianBlur(img_proc, img_proc, cv::Size(3, 3), 5);
    //cv::threshold(img_proc, img_proc, 180, 255, cv::THRESH_BINARY);

    //cv::imshow("debug", img_proc);
    //cv::waitKey(0);

    std::vector<cv::Vec3f> circles;

    int sum = 0;
    cv::HoughCircles(img_proc, circles, cv::HOUGH_GRADIENT, 1, mindist / k, param1, param2 / k, minRadius / k, maxRadius / k);
    if (circles.size() == 0) {
        printf("HoughCircles not find ball!\n");
        printf("No ball found! param2:%.1f\n", param2);
        return -1;
    }
    else if (circles.size() == 1) {
        ball_pos.push_back(circles[0]);

        ball_pos[0][0] *= k;
        ball_pos[0][1] *= k;
        ball_pos[0][2] = (float)(2 * k * ball_pos[0][2] + k);
    }
    else {
        double ballRadius = 24.0;
        double minError;
        int index;
        minError = fabs(circles[0][2] - ballRadius / k);
        index = 0;
        for (int i = 1;i < (int)circles.size();i++) {
            if (fabs(circles[i][2] - ballRadius / k) < minError) {
                minError = fabs(circles[i][2] - ballRadius / k);
                index = i;
            }
        }
        ball_pos.push_back(circles[index]);

        ball_pos[0][0] *= k;
        ball_pos[0][1] *= k;
        ball_pos[0][2] = (float)(2 * k * ball_pos[0][2] + k);
    }

    //cv::Mat img_disp = img_clp.clone();
    //cv::circle(img_disp, cv::Point(ball_pos[0][0], ball_pos[0][1]), 2, cv::Scalar(0), 2);
    //cv::imshow("Proc Image", img_disp);
    //cv::waitKey(0);

    int x, y, width, height;
    x = cvRound(ball_pos[0][0] - ball_pos[0][2]);
    if (x < 0)
        x = 0;
    else if (x >= img_clp.cols)
        x = img_clp.cols - 1;

    y = cvRound(ball_pos[0][1] - ball_pos[0][2]);
    if (y < 0)
        y = 0;
    else if (y >= img_clp.rows)
        y = img_clp.rows - 1;

    width = cvRound(2 * ball_pos[0][2]);
    if (x + width < 0)
        width = 0;
    else if (x + width > img_clp.cols)
        width = img_clp.cols - 1 - x;

    height = cvRound(2 * ball_pos[0][2]);
    if (y + height < 0)
        height = 0;
    else if (y + height > img_clp.rows)
        height = img_clp.rows - 1 - y;

    cv::Rect rect(x, y, width, height);

    cv::Mat img_ballclp;
    img_ballclp = img_clp(rect);

    //cv::imshow("img_ballclp", img_ballclp);
    //cv::waitKey(0);

    cv::threshold(img_ballclp, img_ballclp, 160, 255, cv::THRESH_BINARY);

    //cv::imshow("Proc Image", img_ballclp);
    //cv::waitKey(0);

    std::vector<cv::Vec3f> temp_pos;
    temp_pos.push_back(cv::Vec3f(0, 0, 0));
    sum = 0;
    for (int i = 0;i < img_ballclp.rows; i++) {
        for (int j = 0;j < img_ballclp.cols;j++) {
            if (img_ballclp.at<uchar>(i, j) == 255) {
                temp_pos[0][0] += j;
                temp_pos[0][1] += i;
                sum++;
            }
        }
    }
    temp_pos[0][0] /= sum;
    temp_pos[0][1] /= sum;

    ball_pos[0][0] = x + temp_pos[0][0];
    ball_pos[0][1] = y + temp_pos[0][1];
    ball_pos[0][2] = temp_pos[0][2];




    //ball_pos.clear();

    ////cv::imwrite("D:/pic1.png", img_ballclp);

    //cv::HoughCircles(img_ballclp, ball_pos, cv::HOUGH_GRADIENT, 1, mindist, 200, 40);
    //if (ball_pos.size() != 1) {
    //    ball_pos.push_back(cv::Vec3f(0, 0, 0));
    //    return -1;
    //}

    //cv::circle(img_ballclp, cv::Point(ball_pos[0][0], ball_pos[0][1]), ball_pos[0][2], cv::Scalar(128), 2);
    ////cv::imshow("debug", img_ballclp);
    ////cv::waitKey(1);

    //ball_pos[0][0] += x;
    //ball_pos[0][1] += y;

    return 0;
}

int MyCamera::getBallPnt2(cv::Mat img_clp, std::vector<cv::Vec3f>& ball_pos, double mindist, double param1, double param2, int minRadius, int maxRadius) {

    cv::Mat img_proc = img_clp.clone();

    int sum = 0;
    int times = 0;
    int maxiter = 5;
    while (true) {
        cv::HoughCircles(img_proc, ball_pos, cv::HOUGH_GRADIENT, 1, mindist, param1, param2, minRadius, maxRadius);
        if (ball_pos.size() == 0) {
            printf("HoughCircles not find ball!\n");
            param2 /= 1.2;
            printf("Too many balls found! param2:%.1f\n", param2);
            //return -1;
        }
        else if (ball_pos.size() == 1) {
            break;
        }
        else {
            param2 *= 1.1;
            printf("Too many balls found! param2:%.1f\n", param2);
        }
        if (++times > maxiter) {
            printf("No Ball found!\n");
            return -1;
        }
    }

    return 0;
}

int MyCamera::getBallPnt3(cv::Mat img_clp, std::vector<cv::Vec3f>& ball_pos)
{
    cv::Mat img_proc;
    //cv::imshow("debug1", img_proc);
    //cv::waitKey(0);

    cv::threshold(img_clp, img_proc, 180, 255, cv::THRESH_BINARY);

    //cv::imshow("debug2", img_proc);
    //cv::waitKey(0);

    cv::Vec3f temp_pos(0, 0, 0);
    int sum = 0;
    for (int i = 0;i < img_proc.rows; i++) {
        for (int j = 0;j < img_proc.cols;j++) {
            if (img_proc.at<uchar>(i, j) == 255) {
                temp_pos[0] += j;
                temp_pos[1] += i;
                sum++;
            }
        }
    }
    temp_pos[0] /= sum;
    temp_pos[1] /= sum;

    ball_pos.push_back(temp_pos);

    return 0;
}

int MyCamera::removeColumn(cv::Mat &matrix, unsigned int colToRemove)
{
    if (colToRemove < 0 || colToRemove >= (uint)matrix.cols || matrix.cols <= 0) {
        return -1;
    }

    cv::Mat tmp(matrix.rows, matrix.cols - 1, CV_64FC1);
    
    int j = 0;
    for (int i = 0;i < matrix.cols;i++) {
        if (i != colToRemove){
            matrix.col(i).copyTo(tmp.col(j));
            j++;
        }
    }
    matrix = tmp.clone();

    return 0;
}

int MyCamera::findNearest(cv::Mat &p_img, cv::Mat &p_obj, double thres) {

    cv::Mat p_img_tmp;
    cv::Mat p_obj_tmp;

    cv::Rect rectR(0, 0, 2, 2), rectP(2, 0, 1, 2);
    cv::Mat R = trans(rectR);
    cv::Mat P = trans(rectP);
    cv::Mat p_obj2d = p_obj.rowRange(0, 2);
    cv::Mat p_obj_trans = R * p_obj2d + P * cv::Mat::ones(1, p_obj.cols, CV_64FC1);
    int match_num = 0;

    double dist, mindist;
    unsigned int minindex;
    int rtn = 0;
    for (int i = 0; i < p_img.cols; ) {
        mindist = cv::norm(p_img.col(i) - p_obj_trans.col(i));
        minindex = 0;
        for (int j = 1;j < p_obj.cols; j++) {

            dist = cv::norm(p_img.col(i) - p_obj_trans.col(j));
            if (dist < mindist) {
                mindist = dist;
                minindex = j;
            }
        }
        if (mindist < thres) {
            //p_img.col(i).copyTo(p_img_tmp.col(match_num));
            //p_obj.col(minindex).copyTo(p_obj_tmp.col(match_num));

            p_img_tmp.push_back(p_img.col(i).t());
            p_obj_tmp.push_back(p_obj.col(minindex).t());

            match_num++;

            rtn |= removeColumn(p_img, i);
            rtn |= removeColumn(p_obj, minindex);
            rtn |= removeColumn(p_obj_trans, minindex);
            if (rtn) {
                return -1;
            }
        }
        else {
            i++;
        }
    }

    p_img = p_img_tmp.t();
    p_obj = p_obj_tmp.t();

    return 0;
}

int MyCamera::getBestTrans(cv::Mat p_view, cv::Mat p_world) {

    int n = p_world.cols;
    if (n != p_view.cols) {
        printf("Columns not equal\n");
        return -1;
    }
    
    double sx_w = cv::sum(p_world.row(0))[0];
    double sx2_w = cv::sum(p_world.row(0).mul(p_world.row(0)))[0];
    double sy_w = cv::sum(p_world.row(1))[0];
    double sy2_w = cv::sum(p_world.row(1).mul(p_world.row(1)))[0];
    double sxy_ww = cv::sum(p_world.row(0).mul(p_world.row(1)))[0];

    double sx_v = cv::sum(p_view.row(0))[0];
    double sxx_vw = cv::sum(p_view.row(0).mul(p_world.row(0)))[0];
    double sxy_vw = cv::sum(p_view.row(0).mul(p_world.row(1)))[0];
    double sy_v = cv::sum(p_view.row(1))[0];
    double syy_vw = cv::sum(p_view.row(1).mul(p_world.row(1)))[0];
    double sxy_wv = cv::sum(p_view.row(1).mul(p_world.row(0)))[0];

    cv::Matx33d A;
    cv::Matx31d b1, b2;
    cv::Mat t1, t2;

    A << sx2_w, sxy_ww, sx_w,
        sxy_ww, sy2_w, sy_w,
        sx_w, sy_w, n;

    b1 << sxx_vw,
        sxy_vw,
        sx_v;

    b2 << sxy_wv,
        syy_vw,
        sy_v;

    t1 = cv::Mat(A.inv() * b1);
    t2 = cv::Mat(A.inv() * b2);

    //std::cout << Trans << std::endl;

    cv::Mat t1t = t1.t();
    cv::Mat t2t = t2.t();

    t1t.copyTo(trans.row(0));
    t2t.copyTo(trans.row(1));

    //std::cout << Trans << std::endl;

    return 0;
}