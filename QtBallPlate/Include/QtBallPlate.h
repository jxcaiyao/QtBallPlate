#pragma once

#include <QtWidgets/QWidget>

#include <Windows.h>

#include "ui_QtBallPlate.h"

#include <gts.h>

#include "ControllerCasPID.h"
#include "ControllerPID.h"
#include "MilVision.h"
#include "MotionControl.h"
#include "MyCamera.h"

class QtBallPlate : public QWidget
{
    Q_OBJECT

public:
    CMilVision m_MilVision;		//用于连接相机驱动板
    MyCamera m_MyCamera;		//用于标定与坐标计算

    MotionControl m_XMotCtrl;
    MotionControl m_YMotCtrl;

    ControllerPID m_XCtrlPID;
    ControllerPID m_YCtrlPID;

    ControllerCasPID m_XCtrlCasPID;
    ControllerCasPID m_YCtrlCasPID;

    cv::Mat mask;
    cv::Mat img_raw;			//原始的图像
    cv::Mat img_proc;			//处理的图像
    cv::Mat img_disp;			//显示的图像
    cv::Point2d m_BallPos;		//小球世界坐标
    cv::Point m_BallImg;        //小球图像坐标

public:
    QString m_BallXPosText;
    QString m_BallYPosText;
    QString m_SampleTimeText;
    QString m_DetectTimeText;
    QString m_XCtrlText;
    QString m_YCtrlText;
    QString m_KpText;
    QString m_KiText;
    QString m_KdText;
    QString m_XExptText;
    QString m_YExptText;

public:
    QtBallPlate(QWidget *parent = Q_NULLPTR);

public:
    int Array2Mat(BYTE* array, cv::Mat& img, int row, int col);

private:
    Ui::QtBallPlateClass ui;
};
