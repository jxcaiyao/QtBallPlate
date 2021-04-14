#if !defined(AFX_DISPVISION_H__2AEC5223_77CA_43DE_BE96_6C6FF87C69BC__INCLUDED_)
#define AFX_DISPVISION_H__2AEC5223_77CA_43DE_BE96_6C6FF87C69BC__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000
// MilVision.h : header file

#include "Mil.h"
#include <math.h>

/////////////////////////////////////////////////////////////////////////////
// CDispVision window

class CMilVision
{
// Construction
public:
    CMilVision();

// Attributes
public:

// Operations
public:

// Overrides
    // ClassWizard generated virtual function overrides
    //{{AFX_VIRTUAL(CMilVision)
    //}}AFX_VIRTUAL

// Implementation
public:	
    BYTE *m_pDataArray;          // the buffer of the image array
    BYTE *m_pProcDataArray;      // the buffer of the image array having been processed
    void InitialVision();
    void ImageFrameGrab();
    void GrabContinuous(HWND m_MyView);
    void MiLDestroy();		
    
    virtual ~CMilVision();

private:

    MIL_ID		MilApplication,  /* MIL Application identifier.  */
                MilSystem,       /* MIL System identifier.       */
                MilDisplay,      /* MIL Display identifier.      */
                MilDigitizer,    /* MIL Digitizer identifier.    */
                MilImage;        /* MIL Image buffer identifier. */
////////////////////////////////////////////////////////////////////////////
    // Generated message map functions
protected:
    //{{AFX_MSG(CMilVision)
        // NOTE - the ClassWizard will add and remove member functions here.
    //}}AFX_MSG
};
/////////////////////////////////////////////////////////////////////////////

//{{AFX_INSERT_LOCATION}}
// Microsoft Visual C++ will insert additional declarations immediately before the previous line.

#endif // !defined(AFX_DISPVISION_H__2AEC5223_77CA_43DE_BE96_6C6FF87C69BC__INCLUDED_)
