// TraOutlier.h : main header file for the TraOutlier application
//
#pragma once

#ifndef __AFXWIN_H__
	#error "include 'stdafx.h' before including this file for PCH"
#endif

#include "resource.h"       // main symbols
#include "param.h"			// parameters for the outlier detection algorithm (added by Jae-Gil Lee)


// CTraOutlierApp:
// See TraOutlier.cpp for the implementation of this class
//

class CTraOutlierApp : public CWinApp
{
public:
	CTraOutlierApp();


// Overrides
public:
	virtual BOOL InitInstance();

// Implementation
	afx_msg void OnAppAbout();
	DECLARE_MESSAGE_MAP()
};

extern CTraOutlierApp theApp;