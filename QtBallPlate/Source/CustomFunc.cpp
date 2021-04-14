#include "CustomFunc.h"

#include <thread>

#include <qmessagebox.h>
#include <qdir.h>
#include <qfile.h>
#include <qtextstream.h>


bool isGrab = false;

double CPUFrequency = 3600000000;

inline UINT64 GetCycleCount()
{
    __asm
    {
        _emit 0x0F;
        _emit 0x31;
    }
}

void GetCPUFrequency()
{
    UINT64 start, end;

    start = GetCycleCount();
    end = GetCycleCount() - start;
    start = GetCycleCount();

    Sleep(1000);
    CPUFrequency = (double)(GetCycleCount() - start - end);
}

int CommandHandler(QString str, int error)
{
    if (!error) {
        return 0;
    }
    QString str1;
    str1.sprintf(" : %d", error);
    str1 = str + str1;
    QMessageBox::warning(NULL, "WARNING", str1);

    return error;
}

void SaveData(QtBallPlate* proc) {

    time_t tim = time(NULL);
    tm LocTime;
    localtime_s(&LocTime, &tim);

    QString folderPath;
    folderPath.sprintf("../data/%d_%d_%d",
        LocTime.tm_year + 1900,
        LocTime.tm_mon + 1,
        LocTime.tm_mday);

    QDir dir;
    if (!dir.exists(folderPath)) {
        if (!dir.mkdir(folderPath)) {
            QMessageBox::warning(NULL,"Failed", QString("Failed to Create New Directory:\n") + folderPath);
            return;
        }
    }

    QString fileName;
    fileName.sprintf("/data_%d_%d_%d_%d_%d_%d.csv",
        LocTime.tm_year + 1900,
        LocTime.tm_mon + 1,
        LocTime.tm_mday,
        LocTime.tm_hour,
        LocTime.tm_min,
        LocTime.tm_sec);
    fileName = folderPath + fileName;

    QFile file(fileName);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
        QMessageBox::warning(NULL, "Failed", QString("Failed to Open File:\n") + fileName);
        return;
    }

    QTextStream fout(&file);
    fout << "t,xexpt,yexpt,xpos,ypos,xctrl,yctrl,xenc,yenc,xt,yt" << endl;

    UINT64 t3, t4;
    double XEnc, YEnc;

    t3 = GetCycleCount();
    while (true) {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));

        GT_GetEncPos(1, &XEnc);
        GT_GetEncPos(2, &YEnc);

        t4 = GetCycleCount();
        double time = (t4 - t3) * 1000 / CPUFrequency;
        fout << time << "," 
            //<< proc->m_XCtrlPID.GetExpt() << "," 
            //<< proc->m_YCtrlPID.GetExpt() << ","
            << proc->m_XCtrlCasPID.posCtrl.GetExpt() << ","
            << proc->m_YCtrlCasPID.posCtrl.GetExpt() << ","
            << proc->m_BallPos.x << "," 
            << proc->m_BallPos.y << "," 
            << proc->m_XCtrlPID.GetOutput() << "," 
            << proc->m_YCtrlPID.GetOutput() << "," 
            << XEnc << "," 
            << YEnc << "," 
            << proc->m_XCtrlPID.GetTimeIntvel() << "," 
            << proc->m_YCtrlPID.GetTimeIntvel() << endl;
    }
}

void SaveData2(double sampleTime, QtBallPlate* proc) {
    static bool isFirst = true;
    static QFile* file;

    if (isFirst) {
        time_t tim = time(NULL);
        tm LocTime;
        localtime_s(&LocTime, &tim);

        QString folderPath;
        folderPath.sprintf("../IdentifyData/%d_%d_%d",
            LocTime.tm_year + 1900,
            LocTime.tm_mon + 1,
            LocTime.tm_mday);

        QDir dir;
        if (!dir.exists(folderPath)) {
            if (!dir.mkdir(folderPath)) {
                QMessageBox::warning(NULL, "Failed", QString("Failed to Create New Directory:\n") + folderPath);
                return;
            }
        }

        QString fileName;
        fileName.sprintf("/IdData_%d_%d_%d_%d_%d_%d.csv",
            LocTime.tm_year + 1900,
            LocTime.tm_mon + 1,
            LocTime.tm_mday,
            LocTime.tm_hour,
            LocTime.tm_min,
            LocTime.tm_sec);
        fileName = folderPath + fileName;

        file = new QFile(fileName);

        if (!file->open(QIODevice::WriteOnly | QIODevice::Text)) {
            QMessageBox::warning(NULL, "Fail", "Fail to open file:\n" + fileName);
        }
         
        QTextStream fOut(file);

        fOut << "t/s,xpos/mm,xenc" << endl;

        isFirst = false;
        return;
    }

    static double t0, xpos0;
    static bool isInit = false;
    if (fabs(proc->m_BallPos.x) >= 1e-8) {

        double t1;
        double xenc;
        GT_GetEncPos(1, &xenc);

        if (!isInit) {
            t0 = sampleTime;
            t1 = t0;
            xpos0 = proc->m_BallPos.x;
            isInit = true;
        }
        else {
            t1 = sampleTime;
        }

        QTextStream fOut(file);
        fOut<< t1 - t0 << ","
            << proc->m_BallPos.x - xpos0 << ","
            << xenc << endl;

    }
}

int DetectControl(QtBallPlate* proc)
{
    UINT64 t1, t2;

    t1 = GetCycleCount();
    proc->Array2Mat(proc->m_MilVision.m_pDataArray, proc->img_raw, 576, 768);

    double xEnc, yEnc;
    GT_GetEncPos(1, &xEnc);
    GT_GetEncPos(2, &yEnc);

    double xAng = 0.0, yAng = 0.0;
    proc->m_XMotCtrl.CalcPlateAng(xEnc, &xAng);
    proc->m_YMotCtrl.CalcPlateAng(yEnc, &yAng);

    if (proc->m_MyCamera.getBallPosition(proc->m_BallPos, proc->m_BallImg, proc->img_raw, xAng, yAng)) {
        //AfxMessageBox(TEXT("No Ball Detected!"));
        proc->img_raw.copyTo(proc->img_disp);

        return -1;
    }

    t2 = GetCycleCount();
    proc->m_DetectTimeText.sprintf("%.1fms", (t2 - t1) * 1000.0 / CPUFrequency);

    proc->m_BallXPosText.sprintf("%.1fmm", proc->m_BallPos.x);
    proc->m_BallYPosText.sprintf("%.1fmm", proc->m_BallPos.y);
    proc->m_XCtrlText.sprintf("%.0f\t%.0f\t%.0f\t%.0f\t%.0f", 
        proc->m_XCtrlCasPID.GetOutput(), 
        xEnc, 
        proc->m_XCtrlCasPID.velCtrl.GetP(), 
        proc->m_XCtrlCasPID.velCtrl.GetI(),
        proc->m_XCtrlCasPID.velCtrl.GetD());
        proc->m_YCtrlText.sprintf("%.0f\t%.0f\t%.0f\t%.0f\t%.0f", 
        proc->m_YCtrlCasPID.GetOutput(), 
        yEnc, 
        proc->m_YCtrlCasPID.velCtrl.GetP(), 
        proc->m_YCtrlCasPID.velCtrl.GetI(),
        proc->m_YCtrlCasPID.velCtrl.GetD());

    proc->m_MyCamera.drawPoint(proc->img_raw, proc->img_disp, proc->m_BallImg, cv::Scalar(0));



    /////////////////////////////////////////////////////////////////////////////////////////////////////////
    //普通PID控制

    //double t;
    //t = double(GetCycleCount()) / CPUFrequency;
    //
    //proc->m_XCtrlPID.ControlUpdate(proc->m_BallPos.x, t);
    //proc->m_YCtrlPID.ControlUpdate(proc->m_BallPos.y, t);

    //GT_CrdClear(1, 0);
    //GT_LnXY(1, (long)proc->m_XCtrlPID.GetOutput(), (long)proc->m_YCtrlPID.GetOutput(), 40, 1.0);
    //GT_CrdStart(1, 0);

    //////////////////////////
    //串级PID控制
     
    double t;
    t = double(GetCycleCount()) / CPUFrequency;

    proc->m_XCtrlCasPID.ControlUpdate(proc->m_BallPos.x, t);
    proc->m_YCtrlCasPID.ControlUpdate(proc->m_BallPos.y, t);
    GT_CrdClear(1, 0);
    GT_LnXY(1, (long)proc->m_XCtrlCasPID.GetOutput(), (long)proc->m_YCtrlCasPID.GetOutput(), 40, 1.0);
    GT_CrdStart(1, 0);

    //////////////////////////////////////////////////


    //static int pulse = 1200;
    //static bool isForward = true;
    //GT_CrdClear(1, 0);
    //GT_LnXY(1, pulse, 0, 40, 1);
    //GT_CrdStart(1, 0);

    //if (isForward == true) {
    //    pulse += 10;
    //}
    //else {
    //    pulse -= 10;
    //}

    //if (pulse > 500) {
    //    isForward = false;
    //    pulse -= 10;
    //}
    //else if(pulse < -500){
    //    isForward = true;
    //    pulse += 10;
    //}

    //SaveData2(t1 * 1.0 / CPUFrequency, proc);

    return 0;
}
