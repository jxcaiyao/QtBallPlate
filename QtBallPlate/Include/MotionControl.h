#pragma once

#include <Windows.h>
#include "gts.h"

#define  PI           3.1415926
#define  ENC_1000     1000.0
#define  GEAR_RATIO    8//21//10.0

#define  gPlateRadius  0.098 //mm  
#define  gWheelRadius  0.023 ///mm
#define  gPoleDistance 0.075 //mm//两个基座之间的水平距离
#define  gPoleLength   0.105 //mm//连杆的长度
#define  gWheelHight   0.060 //mm
#define  gPlateCenterHight 0.165 //mm

class MotionControl
{
public:

    MotionControl();
    ~MotionControl();

    static void InitMotor(void);
    void CalcPlateAng(double encPos, double* plateAng);

    BOOL AxisCaptHomeWithoutLimit(int axisno, double HomeVel, double HomeAcc, long routePos);
};

