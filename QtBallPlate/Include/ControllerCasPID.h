#pragma once

#include "ControllerPID.h"

typedef struct {
    TPIDParams posCtrlParams;
    TPIDParams velCtrlParams;
} TCasPIDParams;

class ControllerCasPID {

public:
    ControllerPID posCtrl;
    ControllerPID velCtrl;

    ControllerCasPID();
    ControllerCasPID(TCasPIDParams casPIDParams);
    ControllerCasPID(TPIDParams posCtrlParams, TPIDParams velCtrlParams);
    ~ControllerCasPID();

    void InitParams(TCasPIDParams casPIDParams);
    void InitParams(TPIDParams posCtrlParams, TPIDParams velCtrlParams);

    void ChangeExpect(double expt);
    void ControlUpdate(double expt, double fedbck, double timeNow);
    void ControlUpdate(double fedbck, double timeNow);

    double GetOutput(void);
};