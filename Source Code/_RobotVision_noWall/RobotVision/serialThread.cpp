/********************************************************************
	created:	2002/09/18
	created:	18:9:2002   23:44
	filename: 	C:\Development c++\Serial communication\SerialApp\serialThread.cpp
	file path:	C:\Development c++\Serial communication\SerialApp
	file base:	serialThread
	file ext:	cpp
	author:		Chaiyasit T.
	
	purpose:	This class is used to hanle doing thread of serial communcation.
*********************************************************************/
#include "stdafx.h"
#include "afxwin.h"
#include "serialCtl.hpp"
#include "resource.h"
#include "RobotVisionDlg.h"
#include "serialThread.hpp"

const unsigned short MAX_MESSAGE = 100;

IMPLEMENT_DYNCREATE(SerialThread,CWinThread)
//SerialThread::SerialThread()
//----------------------------------------------------------------------------
//Constructor
//
SerialThread::SerialThread()
:ptrDlg(NULL)
{
}

//SerialThread::~SerialThread()
//----------------------------------------------------------------------------
//Deconstructor
//
SerialThread::~SerialThread()
{
  ptrDlg = NULL;
}

//SerialThread::InitInstance()
//----------------------------------------------------------------------------
//Deconstructor
//
BOOL
SerialThread::InitInstance()
{
  return TRUE;
}

// SerialThread::Run()
//----------------------------------------------------------------------------
// Description: This is a virtual function that is called when thread process
//               is created to be one task.
//
int
SerialThread::Run()
{
  // Check signal controlling and status to open serial communication.
  while(1)
  {
    // Start process of serial communication operation.
    while(ptrDlg->activeProccess == TRUE)
    {
      // enter if there is command of openning and port has be closed before.
      if ((SCC::serialCtl().getStatusPort() == FALSE) && 
        ptrDlg->openPortActivate)
      {
        // open port by calling api function of class serialCtl.
        if (SCC::serialCtl().openPort(ptrDlg->configSerial_,
          ptrDlg->m_strNamePort) == TRUE)
        {
          // Indicate message to status moditor that communication connected already.
          ptrDlg->SetDlgItemText(IDC_EDIT_STATUS_PORT,"Connected");
        }
        else
        {
          // Have problem since opening serial communication.
          ptrDlg->activeProccess = FALSE;
        }
      }
      else if (ptrDlg->openPortActivate)
      {
        char mess[MAX_MESSAGE];
		
        unsigned int lenBuff = MAX_MESSAGE;
        unsigned long lenMessage;
        static CString outPut;
		CString data_in(_T("")); 
		
        if (SCC::serialCtl().read_scc(mess,lenBuff,lenMessage) == TRUE)
        {
			
			if (lenMessage > 0)
			{
            outPut =  mess;
			for(int i = 0; i < lenMessage; i++)
				{
					if(mess[i] == 'X')
					{
						data_in = "";
					}
					else if (mess[i] == 'Y')
					{
						ptrDlg->SetDlgItemText(IDC_EDIT_X_CURRENT, data_in);
						ptrDlg->m_dbXCurrent = atof(data_in);
						data_in = "";
					}
					else if(mess[i] == 'A')
					{	
						ptrDlg->SetDlgItemText(IDC_EDIT_Y_CURRENT, data_in);
						ptrDlg->m_dbYCurrent = atof(data_in);
						data_in = "";
					}
					else if(mess[i] == 'N')
					{
						ptrDlg->SetDlgItemText(IDC_EDIT_ANGLE_CURRENT, data_in);
						ptrDlg->m_dbAngleCurrent = atof(data_in);
						ptrDlg->doneRobot = true;
						data_in = "";
					}
					else if(mess[i] == 'T')
					{
						ptrDlg->SetDlgItemText(IDC_EDIT_ANGLE_CURRENT, data_in);
						ptrDlg->m_dbAngleCurrent = atof(data_in);
					//	ptrDlg->doneRobot = true;
						data_in = "";
					}
					else if(mess[i] == 'D')
					{
						ptrDlg->m_dbDisTemp = atof(data_in);
						data_in = "";
					}
					else data_in += mess[i];
				}
		//	ptrDlg->m_strtest = outPut;//ssk.str().c_str();
		//	ptrDlg->SetDlgItemText(IDC_EDIT_TEXT_RECEIVE, outPut);
          }
        }
        else
        {
          ptrDlg->activeProccess = FALSE;
        }
      }
  
      // Check signal controlling to send data.
      if (ptrDlg->sendActivate && (ptrDlg->m_strTextSend.GetLength() > 0))
      {
        unsigned long len;
        SCC::serialCtl().write_scc(ptrDlg->m_strTextSend ,
          ptrDlg->m_strTextSend.GetLength(),len);
        ptrDlg->sendActivate = false;
        ptrDlg->SetDlgItemText(IDC_EDIT_TEXT_SEND,"");
		
      }
  
      // Check status and signal controlling to close serial communication.
      if (ptrDlg->closePortActivate)
      {
        if (SCC::serialCtl().closePort() == TRUE)
        {
          // Show message that close when performing of closing port okay.
          ptrDlg->SetDlgItemText(IDC_EDIT_STATUS_PORT,"Closed");
          ptrDlg->closePortActivate = false;
        }
      }
    }
  }
  return 0;
}
/****************************End of file**************************************/