#include "ControllerPID.h"

#include <Windows.h>

#include <qmessagebox.h>
#include <qstring.h>
#include <qtextstream.h>
#include <qdir.h>
#include <qfile.h>


ControllerPID::ControllerPID() :
    expt(0),
    exptLast(0),
    exptMax(0),
    exptFilterCoeff(0),
    fedbck(0),
    fedbckLast(0),
    err(0),
    errLast(0),
    errSum(0),
    timeNow(0),
    timeLast(0),
    timeIntvel(0),
    timeIntvelMax(0),
    timePast(0),
    output(0),
    outputLast(0),
    outputMax(0),
    outputFilterCoeff(0),
    dOutputMax(0),
    kp(0),
    ki(0),
    kd(0),
    kf(0),
    P(0),
    I(0),
    D(0),
    F(0),
    IMax(0),
    ISeperateErr(0),
    DMax(0),
    isLog(false),
    controllerName(""),
    logFileName("")
{

};

ControllerPID::ControllerPID(TPIDParams PIDParams) 
{
    InitParams(PIDParams);
}

ControllerPID::~ControllerPID() 
{

}

void ControllerPID::InitParams(TPIDParams PIDParams) 
{
    expt = PIDParams.expt;
    exptLast = expt;
    exptMax = PIDParams.exptMax;
    exptFilterCoeff = PIDParams.exptFilterCoeff;

    fedbck = PIDParams.fedbck;
    fedbckLast = fedbck;

    err = expt - fedbck;
    errLast = err;
    errSum = 0;

    timeNow = PIDParams.timeNow;
    timeLast = timeNow;
    timeIntvel = 0;
    timeIntvelMax = PIDParams.timeIntvelMax;
    timePast = 0;

    output = 0;
    outputLast = 0;
    outputMax = PIDParams.outputMax;
    outputFilterCoeff = PIDParams.outputFilterCoeff;
    dOutputMax = PIDParams.dOutputMax;

    kp = PIDParams.kp;
    ki = PIDParams.ki;
    kd = PIDParams.kd;
    kf = PIDParams.kf;

    P = 0;
    I = 0;
    D = 0;
    F = 0;

    IMax = PIDParams.IMax;
    ISeperateErr = PIDParams.ISeperateErr;

    DMax = PIDParams.DMax;

    isLog = PIDParams.isLog;
    controllerName = PIDParams.controllerName;
    logFileName = PIDParams.logFileName;

    if (isLog) {
        CreateLogFile();
    }
}

void ControllerPID::ControlUpdate(double expt, double fedbck, double timeNow) 
{
    ChangeExpect(expt);
    ControlUpdate(fedbck, timeNow);
}

void ControllerPID::ControlUpdate(double fedbck, double timeNow)
{
    timeLast = this->timeNow;
    this->timeNow = timeNow;
    timeIntvel = this->timeNow - timeLast;
    timePast += timeIntvel;

    /////////////
    timeIntvel = 0.08;
    /////////////

    //if (timeIntvel > timeIntvelMax) {
    //    return;
    //}

    fedbckLast = this->fedbck;
    this->fedbck = fedbck;

    errLast = err;
    err = this->expt - this->fedbck;
    errSum += err * (fabs(err) < ISeperateErr ? 1 : 0);
    errSum = Saturation(errSum, IMax / ki / timeIntvel);

    P = kp * err;
    I = ki * timeIntvel * errSum;
    D = kd / timeIntvel * (err - errLast);
    F = kf * GetExptVel();

    I = Saturation(I, IMax);
    //if (fabs(err) > ISeperateErr) {
    //    I = 0;
    //}

    D = Saturation(D, DMax);

    double tmpOut, dout;
    tmpOut = P + I + D + F;
    tmpOut = Saturation(tmpOut, outputMax);

    dout = outputFilterCoeff * (tmpOut - output);
    dout = Saturation(dout, dOutputMax);
    output += dout;

    if (isLog) {
        LogDebugData();
    }
}

void ControllerPID::ChangeExpect(double expt)
{
    exptLast = this->expt;
    this->expt += exptFilterCoeff * (Saturation(expt, exptMax) - this->expt);
}

void ControllerPID::ChangePIDGain(double kp, double ki, double kd)
{
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
}

double ControllerPID::GetExpt(void)
{
    return expt;
}

double ControllerPID::GetOutput(void)
{
    return output;
}

double ControllerPID::GetKp(void)
{
    return kp;
}

double ControllerPID::GetKi(void)
{
    return ki;
}

double ControllerPID::GetKd(void)
{
    return kd;
}

double ControllerPID::GetP(void)
{
    return P;
}

double ControllerPID::GetI(void)
{
    return I;
}

double ControllerPID::GetD(void)
{
    return D;
}

double ControllerPID::GetTimeIntvel(void)
{
    return timeIntvel;
}

double ControllerPID::GetExptVel(void)
{
    return (expt - exptLast) / timeIntvel;
}

double ControllerPID::GetFedBckVel(void)
{
    return (fedbck - fedbckLast) / timeIntvel;
}

double ControllerPID::Saturation(double input, double Max)
{
    return max(min(input, Max), -Max);
}

void ControllerPID::CreateLogFile(void)
{
    QString folderPath;
    folderPath.sprintf("../ControllerData/");
    folderPath = folderPath + controllerName + QString("Data");

    time_t tim = time(NULL);
    tm locTime;
    localtime_s(&locTime, &tim);

    QString timeString;
    timeString.sprintf("/%d_%d_%d",
        locTime.tm_year + 1900,
        locTime.tm_mon + 1,
        locTime.tm_mday);

    folderPath += timeString;
    
    QDir dir;
    if (!dir.exists(folderPath)) {
        if (!dir.mkdir(folderPath)) {
            QMessageBox::warning(NULL, "Fail!", "Fail to make directory:\n" + folderPath);
        }
    }

    QString fileName;
    fileName.sprintf("/%s_%d_%d_%d_%d_%d_%d.csv",
        logFileName.toStdString().c_str(),
        locTime.tm_year + 1900,
        locTime.tm_mon + 1,
        locTime.tm_mday,
        locTime.tm_hour,
        locTime.tm_min,
        locTime.tm_sec);

    logFileName = folderPath + fileName;

    fLog = new QFile(logFileName);

    if (!fLog->open(QIODevice::WriteOnly | QIODevice::Text)) {
        QMessageBox::warning(NULL, "Fail", "Fail to open file:\n" + logFileName);
        return;
    }

    QTextStream fOut(fLog);
    fOut << "time,"
        << "expt,"
        << "fedbck,"
        << "err,"
        << "errSum,"
        << "output,"
        << "P,"
        << "I,"
        << "D,"
        << "F,"
        << "kp,"
        << "ki,"
        << "kd,"
        << "kf,"
        << "exptMax,"
        << "exptFilterCoeff,"
        << "timeIntvelMax,"
        << "outputMax,"
        << "outputFilterCoeff,"
        << "dOutputMax,"
        << "IMax,"
        << "ISeprateErr"
        << endl;

    fOut << ",,,,,,,,,,"
        << kp << ","
        << ki << ","
        << kd << ","
        << kf << ","
        << exptMax << ","
        << exptFilterCoeff << ","
        << timeIntvelMax << ","
        << outputMax << ","
        << outputFilterCoeff << ","
        << dOutputMax << ","
        << IMax << ","
        << ISeperateErr 
        << endl;
}

void ControllerPID::LogDebugData(void)
{
    QTextStream fOut(fLog);
    fOut << timePast << ","
        << expt << ","
        << fedbck << ","
        << err << ","
        << errSum << ","
        << output << ","
        << P << ","
        << I << ","
        << D << ","
        << F << endl;
}
