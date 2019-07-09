
// RobotVisionDlg.h : header file
//

#pragma once
#include "afxwin.h"
//Point cloud include
#include <boost/thread/thread.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/openni_camera/openni_driver.h>
#include <pcl/console/parse.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/ModelCoefficients.h>

#include <pcl/common/time.h>
#include <pcl/common/common.h>
#include "afxcmn.h"

//Type definition
typedef pcl::PointXYZ CloudType;
typedef pcl::PointCloud<CloudType> Cloud;
typedef pcl::PointCloud<CloudType>::Ptr CloudPtr;
typedef pcl::PointCloud<CloudType>::ConstPtr CloudConstPtr;


class SerialThread;

// CRobotVisionDlg dialog	
class CRobotVisionDlg : public CDialogEx
{
// Construction
public:
	CRobotVisionDlg(CWnd* pParent = NULL);	// standard constructor

// Dialog Data
	enum { IDD = IDD_ROBOTVISION_DIALOG };

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);	// DDX/DDV support


// Implementation
protected:
	HICON m_hIcon;

	// Generated message map functions
	virtual BOOL OnInitDialog();
	afx_msg void OnSysCommand(UINT nID, LPARAM lParam);
	afx_msg void OnPaint();
	afx_msg HCURSOR OnQueryDragIcon();
	DECLARE_MESSAGE_MAP()
public:
	CString m_strNamePort;
	CString m_strBaudRate;
	CString m_strTextSend;
	CString m_strTextReceive;
	CString m_strPortStatus;
	CString m_strtest;
	CString m_strXTarget;
	CString m_strYTarget;
	CString m_strXCurrent;
	CString m_strYCurrent;
	CString m_strAngleCurrent;
	double m_dbXCurrent;
	double m_dbYCurrent;
	double m_dbAngleCurrent;
	double m_dbXTarget;
	double m_dbYTarget;
	bool openPortActivate;
	bool closePortActivate;
	bool sendActivate;
	BOOL activeProccess;
	DCB configSerial_;
	SerialThread* serialProcess;
	void UpdateConfig();
	afx_msg void OnBnClickedButtonOpenPort();
	afx_msg void OnBnClickedButtonClosePort();
	afx_msg void OnBnClickedButtonExit();
	afx_msg void OnBnClickedButtonSendData();
	afx_msg void OnBnClickedButtonRun();
	
	void KinectThread();
	static UINT ControllerThread(LPVOID pParam);
	void ControllerThread();

	CSliderCtrl m_sliderTilt;
	int m_angle;
	afx_msg void OnNMCustomdrawSliderTilt(NMHDR *pNMHDR, LRESULT *pResult);
	afx_msg void OnBnClickedButton1();
	void goTarget();
	void updateStatus(void);
	void sendCommand(CString dir, double val);
	void safe_move(void);
	void escapeObstacle(CloudType& max_point, CloudType& min_point, bool Optimal);
	bool doneRobot;
	void waitRobot(void);
	void escapeObstacle2(CloudType& max_point, CloudType& min_point);
	double m_dbDisTemp;
};
