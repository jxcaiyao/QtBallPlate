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
    //metero II��
    m_pDataArray = NULL; 
    m_pProcDataArray = (byte *)malloc(768*576*sizeof(byte));

    MappAlloc(M_DEFAULT, &MilApplication); //ָ��MILӦ��
    MsysAlloc(M_DEF_SYSTEM_TYPE, M_DEF_SYSTEM_NUM, M_DEFAULT, &MilSystem);  //ָ��Ӳ��ϵͳ
    MdigAlloc(MilSystem, M_DEFAULT, TEXT("M_PAL"), M_DEFAULT, &MilDigitizer); 
    MdispAlloc(MilSystem, M_DEF_DISPLAY_NUM, M_DEF_DISPLAY_FORMAT, M_WINDOWED, &MilDisplay);//
    MbufAllocColor(MilSystem, 1, 768, 576, 8+M_UNSIGNED,
             (MilDigitizer? M_IMAGE+M_DISP+M_GRAB : M_IMAGE+M_DISP), &MilImage);
    MdigGrab(MilDigitizer, MilImage);
    if(MbufInquire(MilImage,M_SIZE_BIT,M_NULL)==8)
        MbufInquire(MilImage,M_HOST_ADDRESS,&m_pDataArray);	
    
/*
 //MORPHIS��	
    m_pDataArray = NULL; 
    m_pProcDataArray = (byte *)malloc(768*576*sizeof(byte));

    MappAlloc(M_DEFAULT, &MilApplication); //ָ��MILӦ��
    MsysAlloc(M_SYSTEM_MORPHIS, M_DEF_SYSTEM_NUM, M_DEFAULT, &MilSystem);  //ָ��Ӳ��ϵͳ
    MdispAlloc(MilSystem,M_DEF_DISPLAY_NUM,"M_PAL",M_WINDOWED,&MilDisplay);
    MdigAlloc(MilSystem, M_DEV0, M_DEF_DIGITIZER_FORMAT, M_DEFAULT, &MilDigitizer); 
    MdigChannel(MilDigitizer,M_CH0);//ѡ��ͨ��
    MbufAllocColor(MilSystem, 1, 768, 576, 8+M_UNSIGNED,(MilDigitizer? M_IMAGE+M_DISP+M_GRAB : M_IMAGE+M_DISP), &MilImage);
    MdigGrab(MilDigitizer,MilImage);
    
    if(MbufInquire(MilImage,M_SIZE_BIT,M_NULL)==8)
        MbufInquire(MilImage,M_HOST_ADDRESS,&m_pDataArray);	
    */
}

void CMilVision::GrabContinuous(HWND m_MyView) 
{
    MdigHalt(MilDigitizer);
    MdigGrabContinuous(MilDigitizer, MilImage); //����ץȡ��ʾ
    MdispSelectWindow(MilDisplay, MilImage,m_MyView);//������ʾ����
}

void CMilVision::ImageFrameGrab() 
{//��̬ץͼ
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
