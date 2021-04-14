#include "ControllerCasPID.h"

ControllerCasPID::ControllerCasPID()
{
}

ControllerCasPID::ControllerCasPID(TCasPIDParams casPIDParams)
{
    InitParams(casPIDParams);
}

ControllerCasPID::ControllerCasPID(TPIDParams posCtrlParams, TPIDParams velCtrlParams)
{
    InitParams(posCtrlParams, velCtrlParams);
}

ControllerCasPID::~ControllerCasPID()
{
}

void ControllerCasPID::InitParams(TCasPIDParams casPIDParams)
{
    InitParams(casPIDParams.posCtrlParams, casPIDParams.velCtrlParams);
}

void ControllerCasPID::InitParams(TPIDParams posCtrlParams, TPIDParams velCtrlParams)
{
    posCtrl.InitParams(posCtrlParams);
    velCtrl.InitParams(velCtrlParams);
}

void ControllerCasPID::ChangeExpect(double expt)
{
    posCtrl.ChangeExpect(expt);
}

void ControllerCasPID::ControlUpdate(double expt, double fedbck, double timeNow)
{
    posCtrl.ChangeExpect(expt);
    ControlUpdate(fedbck, timeNow);
}

void ControllerCasPID::ControlUpdate(double fedbck, double timeNow)
{
    posCtrl.ControlUpdate(fedbck, timeNow);
    velCtrl.ControlUpdate(posCtrl.GetOutput(), posCtrl.GetFedBckVel(), timeNow);
}

double ControllerCasPID::GetOutput(void)
{
    return velCtrl.GetOutput();
}
