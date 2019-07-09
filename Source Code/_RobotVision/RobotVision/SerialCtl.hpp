/********************************************************************
	created:	2002/09/01
	created:	1:9:2002   11:05
	filename: 	D:\Applicaiton testing\SerialApp\serialCtl\SerialCtl.hpp
	file path:	D:\Applicaiton testing\SerialApp\serialCtl
	file base:	SerialCtl
	file ext:	hpp
	author:		Chaiyasit T.
	
	purpose:	Use to control the signal of serial communication.
*********************************************************************/

#ifndef SERIAL_CTL_HPP
#define SERIAL_CTL_HPP

// class SerialCtl
//-----------------------------------------------------------------------------
// Description: This class handle the functionality that interface with
// the serial communication.
// 
class SerialCtl
{
public:
  SerialCtl();  // Constructor
  ~SerialCtl(); // Destructor

public:
	void setStatusPort(BOOL on_off);     // set Status port whether no or off. 
	BOOL closePort();                    // close port operator.       


  BOOL openPort(DCB dcb,               // open serial communicaiton port. 
    const char* portName = "COM1");    // Default port is COM1.
	
  BOOL read_scc(char* inputData,       // read data from serial communication.
    const unsigned int& sizeBuffer,    // sizeBuffer is the size of pakcet that
    unsigned long& length);            // receive from serail port.
	
  BOOL write_scc(LPCVOID data,     // write data to serial communication
    const unsigned int& sizeBuffer,    // sizeBufer is the size of packet that     
    unsigned long& length);            // want to send to serial port.

  HANDLE getHandlePort();              // The Entry point to get port's handle. 
  BOOL getStatusPort();                // The entyy point to get port's staus. 

private:
  BOOL statusPort_;                    // port's status.
	HANDLE handlePort_;                  // the object that is a instace of port. 
  DCB config_;                         // configuation of serial communication.  
};

#endif //SERIAL_CTL_HPP

/****************************End of file**************************************/
