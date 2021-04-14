#pragma once

#include <qstring.h>
#include <qfile.h>
#include <math.h>

typedef struct {
    double expt;
    double exptMax;
    double exptFilterCoeff;

    double fedbck;

    double timeNow;
    double timeIntvelMax;

    double outputMax;
    double outputFilterCoeff;
    double dOutputMax;

    double kp;
    double ki;
    double kd;
    double kf;
    
    double IMax;
    double ISeperateErr;

    double DMax;

    bool isLog;
    QString controllerName;
    QString logFileName;
} TPIDParams;

class ControllerPID {
private:
    double expt;
    double exptLast;
    double exptMax;
    double exptFilterCoeff;

    double fedbck;
    double fedbckLast;

    double err;
    double errLast;
    double errSum;

    double timeNow;
    double timeLast;
    double timeIntvel;
    double timeIntvelMax;
    double timePast;

    double output;
    double outputLast;
    double outputMax;
    double outputFilterCoeff;
    double dOutputMax;

    double kp;
    double ki;
    double kd;
    double kf;

    double P;
    double I;
    double D;
    double F;

    double IMax;
    double ISeperateErr;

    double DMax;

    bool isLog;
    QString controllerName;
    QString logFileName;

    QFile* fLog;

public:
    ControllerPID();
    ControllerPID(TPIDParams PIDParams);
    ~ControllerPID();

    void InitParams(TPIDParams PIDParams);
    void ControlUpdate(double expt, double fedbck, double timeNow);
    void ControlUpdate(double fedbck, double timeNow);
    void ChangeExpect(double expt);
    void ChangePIDGain(double kp, double ki, double kd);

public:
    double GetExpt(void);
    double GetOutput(void);
    double GetKp(void);
    double GetKi(void);
    double GetKd(void);
    double GetP(void);
    double GetI(void);
    double GetD(void);
    double GetTimeIntvel(void);
    double GetExptVel(void);
    double GetFedBckVel(void);

private:
    double Saturation(double input, double Max);
    void CreateLogFile(void);
    void LogDebugData(void);
};