#pragma once

#include "QtBallPlate.h"

extern bool isGrab;
extern double CPUFrequency;

extern inline UINT64 GetCycleCount();
extern void GetCPUFrequency();
extern int CommandHandler(QString str, int error);
extern void SaveData(QtBallPlate* proc);
extern int DetectControl(QtBallPlate* proc);