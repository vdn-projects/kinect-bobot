/********************************************************************
  created:  2002/09/01
  created:  1:9:2002   11:05
  filename:   D:\Applicaiton testing\SerialApp\serialCtl\SerialCtl.hpp
  file path:  D:\Applicaiton testing\SerialApp\serialCtl
  file base:  SerialCtl
  file ext: hpp
  author:   Chaiyasit T.
  
  purpose:  Use to control the serial communication's signal.
*********************************************************************/
#include "stdafx.h"
#include "afxwin.h"
#include "serialCtl.hpp"
#include "RobotVision.h"
#include "RobotVisionDlg.h"

// SerialCtl::SerialCtl()
//-----------------------------------------------------------------------------
// Description: Constructor
//
SerialCtl::SerialCtl()
:statusPort_(FALSE),
 handlePort_(NULL)
{
  // default parameter.
  config_.ByteSize = 8;            // Byte of the Data.
  config_.StopBits = ONESTOPBIT;   // Use one bit for stopbit.
  config_.Parity = NOPARITY;       // No parity bit
  config_.BaudRate = CBR_9600;     // Buadrate 9600 bit/sec
}

// SerialCtl::~SerialCtl()
//-----------------------------------------------------------------------------
// Description: Destructor
//
SerialCtl::~SerialCtl()
{
  handlePort_ = NULL;
}

// SerialCtl::openPort(DCB dcb, const char* portName)
//-----------------------------------------------------------------------------
// Description: Open the serial communication port by calling CreateFile
//              function is as the API function. The dcb is a argument 
//              that contain the serial communication configuration. 
//              The portname is as name of device that want to open and perform.
//
BOOL  
SerialCtl::openPort(DCB dcb, const char* portName)
{
  // TODO: Add your control notification handler code here
  if (statusPort_ == false)  // if port is opened already, not open port again.
  {
    handlePort_ = CreateFile(portName,  // Specify port device: default "COM1"
    GENERIC_READ | GENERIC_WRITE,       // Specify mode that open device.
    0,                                  // the devide isn't shared.
    NULL,                               // the object gets a default security.
    OPEN_EXISTING,                      // Specify which action to take on file. 
    0,                                  // default.
    NULL);                              // default.

    // Get current configuration of serial communication port.
    if (GetCommState(handlePort_,&config_) == 0)
    {
      AfxMessageBox("Get configuration port has problem.");
      return FALSE;
    }
    // Assign user parameter.
    config_.BaudRate = dcb.BaudRate;    // Specify buad rate of communicaiton.
    config_.StopBits = dcb.StopBits;    // Specify stopbit of communication.
    config_.Parity = dcb.Parity;        // Specify parity of communication.
    config_.ByteSize = dcb.ByteSize;    // Specify  byte of size of communication.

    // Set current configuration of serial communication port.
    if (SetCommState(handlePort_,&config_) == 0)
    {
       AfxMessageBox("Set configuration port has problem.");
       return FALSE;
    }
  
    // instance an object of COMMTIMEOUTS.
    COMMTIMEOUTS comTimeOut;                   
    // Specify time-out between charactor for receiving.
    comTimeOut.ReadIntervalTimeout = 3;
    // Specify value that is multiplied 
    // by the requested number of bytes to be read. 
    comTimeOut.ReadTotalTimeoutMultiplier = 3;
    // Specify value is added to the product of the 
    // ReadTotalTimeoutMultiplier member
    comTimeOut.ReadTotalTimeoutConstant = 2;
    // Specify value that is multiplied 
    // by the requested number of bytes to be sent. 
    comTimeOut.WriteTotalTimeoutMultiplier = 3;
    // Specify value is added to the product of the 
    // WriteTotalTimeoutMultiplier member
    comTimeOut.WriteTotalTimeoutConstant = 2;
    // set the time-out parameter into device control.
    SetCommTimeouts(handlePort_,&comTimeOut);
    // Updata port's status.
    statusPort_ = TRUE; 
    return TRUE;      
  }
  return FALSE;
}

// SerialCtl::closePort()
//-----------------------------------------------------------------------------
// Description: close communication by destroy handle of communication.
//
BOOL 
SerialCtl::closePort()
{
  if (statusPort_ == TRUE)               // Port need to be open before.
  {
    statusPort_ = false;                 // Update status
    if(CloseHandle(handlePort_) == 0)    // Call this function to close port.
    {
      AfxMessageBox("Port Closeing isn't successed.");
      return FALSE;
    }    
    return TRUE;
  }
  return FALSE;
}

// read_scc(char* inputData,unsigned int sizeBuffer,unsigned int lengh)
//-----------------------------------------------------------------------------
// Description: read data from serial communication port.
//
BOOL
SerialCtl::read_scc(char* inputData,
                    const unsigned int& sizeBuffer,
                    unsigned long& length)
{
  if (ReadFile(handlePort_,  // handle of file to read
    inputData,               // handle of file to read
    sizeBuffer,              // number of bytes to read
    &length,                 // pointer to number of bytes read
    NULL) == 0)              // pointer to structure for data
  {
    AfxMessageBox("Reading of serial communication has problem.");
    return FALSE;
  }
  if (length > 0)
  {
    inputData[length] = NULL; // Assign end flag of message.
    return TRUE;  
  }  
  return TRUE;
}


// SerialCtl::write_scc(const char* outputData,
//                  const unsigned int& sizeBuffer,
//                  unsigned long& length)
//-----------------------------------------------------------------------------
// Description: write the data to serial communicaiton.
//
BOOL
SerialCtl::write_scc(LPCVOID outputData,
                     const unsigned int& sizeBuffer,
                     unsigned long& length)
{
  if (length > 0)
  {
    if (WriteFile(handlePort_, // handle to file to write to
      outputData,              // pointer to data to write to file
      sizeBuffer,              // number of bytes to write
      &length,NULL) == 0)      // pointer to number of bytes written
    {
      AfxMessageBox("Reading of serial communication has problem.");
      return FALSE;
    }
    return TRUE;
  }
  return FALSE;
}


// SerialCtl::getStatusPort()
//-----------------------------------------------------------------------------
// Description: the entry point to get port's status.
//
BOOL
SerialCtl::getStatusPort()
{
  return statusPort_;
}

/****************************End of file**************************************/