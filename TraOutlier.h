// TraOutlier.h : main header file for the TraOutlier application
//
#pragma once

#include "Param.h"			// parameters for the outlier detection algorithm (added by Jae-Gil Lee)


// CTraOutlierApp:
// See TraOutlier.cpp for the implementation of this class
//

class CTraOutlierApp
{
public:
	CTraOutlierApp();


// Overrides
public:
	virtual bool InitInstance();

};

extern CTraOutlierApp theApp;
