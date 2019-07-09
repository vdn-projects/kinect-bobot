/********************************************************************
	created:	2002/09/18
	created:	18:9:2002   21:05
	filename: 	C:\Development c++\Serial communication\SerialApp\serialThread.hpp
	file path:	C:\Development c++\Serial communication\SerialApp
	file base:	serialThread
	file ext:	hpp
	author:		Chaiyasit T.
	
	purpose:	
*********************************************************************/

#ifndef SERIALTHREAD_HPP
#define SERIALTHREAD_HPP

// class SerialThread
//-----------------------------------------------------------------------------
// Description: This class is inherited from CWinThread, use to handle in part 
//              of thread process (multi tasking).
// 


class CRobotVisionDlg;              // Forward Declaration.

class SerialThread : public CWinThread
{
public:
  // To enable objects of CObject-derived classes to be created dynamically at 
  // run time to enable objects of CObject-derived classes to be created 
  // dynamically at run time
  DECLARE_DYNCREATE(SerialThread)  
   
  SerialThread ();               // Constructor.
  virtual ~SerialThread();       // Destructor.
public:
  virtual BOOL InitInstance();   // virtual function that derive from base-class.
  virtual int Run();             // virtual function that derive from base-class.  

  void setOwner(CRobotVisionDlg* ptrDialog)
  {ptrDlg = ptrDialog;}          // Assign the pointer to pointerDlg is as agent 
                                 // of Class CRobotVisionDlg.
private:
  CRobotVisionDlg* ptrDlg;         // The pointer that pointer to object CRobotVisionDlg. 

};


// class SCC
//-----------------------------------------------------------------------------
// Description: This class is as the interface that is used to access the object
//              of Class SerialCtl.
//
class SCC
{
public:
  SCC() {};   // Constructor.
  ~SCC() {};  // Destructor.
public:
  static SerialCtl& serialCtl() // Implement as the singleton funciton.
  {
    static SerialCtl serial;
    return serial;
  }
};


#endif //SERIALTHREAD_HPP

/****************************End of file**************************************/