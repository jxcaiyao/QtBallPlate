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
    CMilVision m_MilVision;		//�����������������
    MyCamera m_MyCamera;		//���ڱ궨���������

    MotionControl m_XMotCtrl;
    MotionControl m_YMotCtrl;

    ControllerPID m_XCtrlPID;
    ControllerPID m_YCtrlPID;

    ControllerCasPID m_XCtrlCasPID;
    ControllerCasPID m_YCtrlCasPID;

    cv::Mat mask;
    cv::Mat img_raw;			//ԭʼ��ͼ��
    cv::Mat img_proc;			//�����ͼ��
    cv::Mat img_disp;			//��ʾ��ͼ��
    cv::Point2d m_BallPos;		//С����������
    cv::Point m_BallImg;        //С��ͼ������

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
