
// RobotVision.h : main header file for the PROJECT_NAME application
//

#pragma once

#ifndef __AFXWIN_H__
	#error "include 'stdafx.h' before including this file for PCH"
#endif

#include "resource.h"		// main symbols


// CRobotVisionApp:
// See RobotVision.cpp for the implementation of this class
//

class CRobotVisionApp : public CWinApp
{
public:
	CRobotVisionApp();

// Overrides
public:
	virtual BOOL InitInstance();

// Implementation

	DECLARE_MESSAGE_MAP()
};

extern CRobotVisionApp theApp;