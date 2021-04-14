#include <math.h>

#include "MotionControl.h"
#include "CustomFunc.h"

MotionControl::MotionControl()
{
}

MotionControl::~MotionControl()
{
}

void MotionControl::InitMotor(void)
{
    int sRtn = 0;
    
    //sRtn = GT_Close();
    //if (CommandHandler("GT_Close", sRtn))
    //    return;

    sRtn = GT_Open();
    if (CommandHandler("GT_Open", sRtn))
        return;

    sRtn = GT_Reset();
    if (CommandHandler("GT_Reset", sRtn))
        return;
    Sleep(500);

    sRtn = GT_LoadConfig("GTS800.cfg");
    if (CommandHandler("GT_LoadConfig", sRtn))
        return;

    sRtn = GT_ClrSts(1, 2);
    if (CommandHandler("GT_ClrSts", sRtn))
        return;

    TPid pid;

    sRtn = GT_GetPid(1, 1, &pid);
    if (CommandHandler("GT_GetPid", sRtn))
        return;

    pid.kp = 0.85;
    pid.ki = 0.00;
    pid.kd = 0.04;

    sRtn = GT_SetPid(1, 1, &pid);
    if (CommandHandler("GT_SetPid", sRtn))
        return;


    sRtn = GT_GetPid(2, 1, &pid);
    if (CommandHandler("GT_GetPid", sRtn))
        return;

    pid.kp = 0.85;
    pid.ki = 0.00;
    pid.kd = 0.04;

    sRtn = GT_SetPid(2, 1, &pid);
    if (CommandHandler("GT_SetPid", sRtn))
        return;

    sRtn = GT_AxisOn(1);
    if (CommandHandler("GT_AxisOn", sRtn))
        return;

    sRtn = GT_AxisOn(2);
    if (CommandHandler("GT_AxisOn", sRtn))
        return;

    Sleep(200);

    TCrdPrm crdPrm;
    memset(&crdPrm, 0, sizeof(crdPrm));
    crdPrm.dimension = 2;
    crdPrm.synVelMax = 40;
    crdPrm.synAccMax = 1.0;
    crdPrm.evenTime = 5;
    crdPrm.profile[0] = 1;
    crdPrm.profile[1] = 2;
    crdPrm.setOriginFlag = true;
    crdPrm.originPos[0] = 0;
    crdPrm.originPos[1] = 0;
    sRtn = GT_SetCrdPrm(1, &crdPrm);
}

void MotionControl::CalcPlateAng(double encPos, double* plateAng)
{
	double beta;
	double ang1, ang2;
	double a, b, c;

	beta = (encPos / (4 * ENC_1000 * GEAR_RATIO)) * 2 * PI;

	a = gPlateRadius;
	b = gPoleLength + gWheelRadius;
	c = sqrt(gPoleDistance * gPoleDistance + (gPlateCenterHight - gWheelHight) * (gPlateCenterHight - gWheelHight));

	ang1 = acos((b * b + c * c - a * a) / (2 * b * c));
	ang2 = atan2((gPlateCenterHight - gWheelHight), gPoleDistance);


	double theta = PI - ang1 - ang2;

	//角度转换到正负180内
	while (beta >= PI)
	{
		beta -= 2 * PI;
	}
	while (beta <= (-PI))
	{
		beta += 2 * PI;
	}

	if ((beta <= theta) && (beta >= (theta - PI)))
	{
		*plateAng = (gWheelRadius / gPlateRadius) * beta;
	}
	else if ((beta >= theta) && (beta <= PI))
	{
		*plateAng = (gWheelRadius / gPlateRadius) * (2 * theta - beta);
	}
	else if ((beta <= (theta - PI)) && (beta >= -PI))
	{
		*plateAng = (gWheelRadius / gPlateRadius) * (-beta + 2 * (theta - PI));
	}

}

//回零
BOOL MotionControl::AxisCaptHomeWithoutLimit(int axisno, double HomeVel, double HomeAcc, long routePos)
{
    //转一圈有约32008个脉冲
	short rtn;
	long pos(0);

	long sts;
	double encpos;
	short capture;
	TTrapPrm trapPrm;
	long homeswt;

	rtn = GT_ClrSts(axisno);
	rtn = GT_SetCaptureMode(axisno, CAPTURE_HOME);
	rtn = GT_PrfTrap(axisno);
	rtn = GT_GetTrapPrm(axisno, &trapPrm);
	trapPrm.acc = HomeAcc;
	trapPrm.dec = HomeAcc;
	rtn = GT_SetTrapPrm(axisno, &trapPrm);
	rtn = GT_SetVel(axisno, HomeVel);
	pos = pos + 50000;
	Sleep(20);

	rtn = GT_SetPos(axisno, pos);
	rtn = GT_Update(1 << (axisno - 1));
	Sleep(50);

    do {
		rtn = GT_GetSts(axisno, &sts);
		rtn = GT_GetCaptureStatus(axisno, &capture, &pos);
        if (0 == (sts & 0x400)) {

			return 1;
		}
    } while (0 == capture);
	rtn = GT_SetPos(axisno, pos + 16500);
	rtn = GT_Update(1 << (axisno - 1));
	Sleep(20);
	do {
		rtn = GT_GetSts(axisno, &sts);;
	} while (sts & 0x400);


	GT_GetDi(MC_HOME, &homeswt);
    sts = homeswt & (1 << (axisno - 1));

    if (sts == 0) {
		rtn = GT_GetEncPos(axisno, &encpos, 1);
		rtn = GT_SetPos(axisno, static_cast<long>(encpos + 7000));
		rtn = GT_Update(1 << (axisno - 1));
		Sleep(50);
		do {
			rtn = GT_GetSts(axisno, &sts);
		} while (sts & 0x400);
	}

	rtn = GT_ClrSts(axisno);
	Sleep(500);
	rtn = GT_GetEncPos(axisno, &encpos, 1);
	pos = static_cast<long>(encpos + 50000);
	Sleep(20);

	rtn = GT_SetCaptureMode(axisno, CAPTURE_HOME);
	rtn = GT_SetPos(axisno, pos);
	rtn = GT_Update(1 << (axisno - 1));
	Sleep(50);

	do {
		rtn = GT_GetSts(axisno, &sts);
		rtn = GT_GetCaptureStatus(axisno, &capture, &pos);
		if (0 == (sts & 0x400)) {

			return 1;
		}
    } while (0 == capture);

	rtn = GT_SetPos(axisno, pos + routePos);
	rtn = GT_Update(1 << (axisno - 1));
	Sleep(20);
	do {
		rtn = GT_GetSts(axisno, &sts);;
	} while (sts & 0x400);

	Sleep(1000);
	GT_SynchAxisPos(1 << (axisno - 1));
	Sleep(20);
	GT_ZeroPos(axisno);
	GT_SetEncPos(axisno, 0);
	rtn = GT_ClrSts(axisno);
	Sleep(10);

	return 0;

}