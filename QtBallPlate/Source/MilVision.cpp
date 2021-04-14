// DispVision.cpp : implementation file
//

#include "QtBallPlate.h"
#include "MilVision.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

/////////////////////////////////////////////////////////////////////////////
// CMilVision

CMilVision::CMilVision()
{
}

CMilVision::~CMilVision()
{
}

/////////////////////////////////////////////////////////////////////////////
// CMilVision message handlers

void CMilVision::InitialVision()
{	/*initialization of image feedback system*/     
    //metero II卡
    m_pDataArray = NULL; 
    m_pProcDataArray = (byte *)malloc(768*576*sizeof(byte));

    MappAlloc(M_DEFAULT, &MilApplication); //指定MIL应用
    MsysAlloc(M_DEF_SYSTEM_TYPE, M_DEF_SYSTEM_NUM, M_DEFAULT, &MilSystem);  //指定硬件系统
    MdigAlloc(MilSystem, M_DEFAULT, TEXT("M_PAL"), M_DEFAULT, &MilDigitizer); 
    MdispAlloc(MilSystem, M_DEF_DISPLAY_NUM, M_DEF_DISPLAY_FORMAT, M_WINDOWED, &MilDisplay);//
    MbufAllocColor(MilSystem, 1, 768, 576, 8+M_UNSIGNED,
             (MilDigitizer? M_IMAGE+M_DISP+M_GRAB : M_IMAGE+M_DISP), &MilImage);
    MdigGrab(MilDigitizer, MilImage);
    if(MbufInquire(MilImage,M_SIZE_BIT,M_NULL)==8)
        MbufInquire(MilImage,M_HOST_ADDRESS,&m_pDataArray);	
    
/*
 //MORPHIS卡	
    m_pDataArray = NULL; 
    m_pProcDataArray = (byte *)malloc(768*576*sizeof(byte));

    MappAlloc(M_DEFAULT, &MilApplication); //指定MIL应用
    MsysAlloc(M_SYSTEM_MORPHIS, M_DEF_SYSTEM_NUM, M_DEFAULT, &MilSystem);  //指定硬件系统
    MdispAlloc(MilSystem,M_DEF_DISPLAY_NUM,"M_PAL",M_WINDOWED,&MilDisplay);
    MdigAlloc(MilSystem, M_DEV0, M_DEF_DIGITIZER_FORMAT, M_DEFAULT, &MilDigitizer); 
    MdigChannel(MilDigitizer,M_CH0);//选择通道
    MbufAllocColor(MilSystem, 1, 768, 576, 8+M_UNSIGNED,(MilDigitizer? M_IMAGE+M_DISP+M_GRAB : M_IMAGE+M_DISP), &MilImage);
    MdigGrab(MilDigitizer,MilImage);
    
    if(MbufInquire(MilImage,M_SIZE_BIT,M_NULL)==8)
        MbufInquire(MilImage,M_HOST_ADDRESS,&m_pDataArray);	
    */
}

void CMilVision::GrabContinuous(HWND m_MyView) 
{
    MdigHalt(MilDigitizer);
    MdigGrabContinuous(MilDigitizer, MilImage); //连续抓取显示
    MdispSelectWindow(MilDisplay, MilImage,m_MyView);//设置显示区域
}

void CMilVision::ImageFrameGrab() 
{//静态抓图
    MdigHalt(MilDigitizer);
    MdigGrab(MilDigitizer, MilImage);
    if(MbufInquire(MilImage,M_SIZE_BIT,M_NULL)==8)
        MbufInquire(MilImage,M_HOST_ADDRESS,&m_pDataArray);
}

void CMilVision::MiLDestroy()
{
    //	free(m_pDataArray);
    //	free(m_pProcDataArray);
    MdigHalt(MilDigitizer);
    MappFreeDefault(MilApplication, MilSystem, MilDisplay,
        MilDigitizer, MilImage);
}
