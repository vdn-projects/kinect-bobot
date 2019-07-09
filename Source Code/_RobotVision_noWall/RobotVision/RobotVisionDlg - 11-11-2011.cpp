// RobotVisionDlg.cpp : implementation file
#include "stdafx.h"
#include "RobotVision.h"
#include "RobotVisionDlg.h"
#include "serialCtl.hpp"
#include "serialThread.hpp"
#include "afxdialogex.h"

#include "nuiMotor.h"
#include "highgui.h"
					
#include <stdlib.h>
#include <math.h>					
#include <iostream>
#include <sstream>

#ifdef _DEBUG
#define new DEBUG_NEW
#endif
#define FPS_CALC(_WHAT_) \
	do \
{ \
	static unsigned count = 0;\
	static double last = pcl::getTime ();\
	double now = pcl::getTime (); \
	++count; \
	if (now - last >= 1.0) \
	{ \
	std::cout << "Average framerate("<< _WHAT_ << "): " << double(count)/double(now - last) << " Hz" <<  std::endl; \
	count = 0; \
	last = now; \
	} \
}while(false)
#ifdef _DEBUG
#define new DEBUG_NEW
#endif
//------------------------------------------------------------------------------------------------------
//Point Cloud declare
//------------------------------------------------------------------------------------------------------
boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
pcl::PassThrough<CloudType> pass_;
pcl::VoxelGrid<CloudType> grid_;
pcl::SACSegmentation<CloudType> seg_;
pcl::ExtractIndices<CloudType> extract_;
pcl::KdTree<CloudType>::Ptr tree ;
pcl::EuclideanClusterExtraction<CloudType> ec;

float closest_x;            
CloudType min_pt, max_pt; //Range of objects' position
double theta = -0.513f; //29.4 degree
const float cosTheta = cos(theta);
const float sinTheta = sin(theta);

//------------------------------------------------------------------------------------------------------
//Controller process declare
//------------------------------------------------------------------------------------------------------
#define RobotWidth 0.44
#define RobotLength 0.365
#define RobotCross 0.422
#define PI  3.1415926535897932

CWinThread *ContrThread;
double anpha, anpha_cmd;
CString direction, direction_old;
double safe_distance, distance_to_target;
unsigned long len;
CloudType maxpointTemp(0,0,0), minpointTemp(0,0,0);
bool  obstacle_flag, bFirst = true;
double m_dbXcurrent_Temp, m_dbYcurrent_Temp, m_dbAngleCurrent_Temp;
int numObs;
//------------------------------------------------------------------------------------------------------
//Test openCV
//------------------------------------------------------------------------------------------------------
CvCapture* capture;
IplImage* frame;

//------------------------------------------------------------------------------------------------------
//Point Cloud local structure
//------------------------------------------------------------------------------------------------------
struct local
{
	CloudConstPtr cloud_;
	boost::mutex cloud_mutex_;

	void cloud_callback(const CloudConstPtr& cloud)
	{
		boost::mutex::scoped_lock lock (cloud_mutex_);
		cloud_ = cloud;    
	}
	CloudConstPtr    getLatestCloud ()
	{

		boost::mutex::scoped_lock lock(cloud_mutex_);
		CloudPtr temp_cloud (new Cloud);
		CloudPtr temp_cloud2 (new Cloud);
		CloudPtr cloud_rotated (new Cloud);
		CloudPtr cloud_pass_ (new Cloud);
		CloudPtr cloud_cluster (new Cloud);
		CloudPtr cloud_cluster_closest (new Cloud);
		CloudPtr cloud_plane (new Cloud);
		std::vector<pcl::PointIndices> cluster_indices;

		pass_.setInputCloud (cloud_);
		pass_.filter (*cloud_pass_);

		grid_.setInputCloud (cloud_pass_);
		grid_.filter (*temp_cloud);
		
		for(Cloud::const_iterator it = temp_cloud->begin(); it != temp_cloud->end(); ++it)
		{
			float x = it->x;
			float y = (it->y)*cosTheta - (it->z)*sinTheta;
			float z = (it->y)*sinTheta + (it->z)*cosTheta + 0.09f;
			y = (y - 0.60f);
			CloudType point;
			point.x = x;
			point.y = - y;
			point.z = z;

			cloud_rotated->push_back(point);
		}


		pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
		pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());

		seg_.setInputCloud (cloud_rotated);
		seg_.segment (*inliers, *coefficients);

		extract_.setInputCloud (cloud_rotated);
		extract_.setIndices (inliers);
		extract_.setNegative (false);
		extract_.filter (*cloud_plane); 

		extract_.setNegative (true);
		extract_.filter (*temp_cloud2);
		ec.setInputCloud(temp_cloud2);
		ec.extract (cluster_indices);

		float closest_z = 2, closest_y = 0, disTemp;
		closest_x = 0;
		min_pt.x = 0;
		min_pt.y = 0;
		min_pt.z = 0;
		max_pt.x = 0;
		max_pt.y = 0;
		max_pt.z = 0;
		
		int countNum = 0;
		
		for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
		{
			CloudPtr cloud_cluster_temp (new Cloud);
			CloudType min_pt_temp(0,0,0);
			CloudType max_pt_temp(0,0,0);
			for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
			{	
				cloud_cluster->points.push_back (temp_cloud2->points[*pit]);
				cloud_cluster_temp->points.push_back (temp_cloud2->points[*pit]);
			}

			pcl::getMinMax3D (*cloud_cluster_temp,  min_pt_temp, max_pt_temp);
			disTemp = min_pt_temp.z;
			
		//	disTemp = cloud_cluster_temp->points[cloud_cluster_temp->points.size()/2].z;

			if(disTemp < closest_z )
			{
				closest_z = disTemp;
				*cloud_cluster_closest = *cloud_cluster_temp;
			}

			countNum ++;

		}
		numObs = countNum;
		viewer->setBackgroundColor (0, 0, 0);
		viewer->removePointCloud("cloud 1");
		viewer->removePointCloud("cloud 2");
		viewer->removePointCloud("cloud 3");
		viewer->deleteText3D("text 3d");
		viewer->deleteText3D("text 3d2");
		viewer->removeShape("Line");
		
		if(countNum)
		{
			pcl::getMinMax3D (*cloud_cluster_closest,  min_pt, max_pt);
			viewer->addLine(max_pt, min_pt, 255, 0, 255, "Line");

			std::stringstream ss1, ss2;
			ss1 << "min_pt "  << min_pt;

			viewer->addText3D(ss1.str().c_str(), min_pt, 0.005, 255, 255, 255, "text 3d");

			ss2 << "max_pt "  << max_pt;
			viewer->addText3D(ss2.str().c_str(), max_pt, 0.005, 255, 255, 255, "text 3d2");

			if((min_pt.x > 0.22) || (max_pt.x < - 0.22) || ( min_pt.z > 0.80) || (min_pt.z <= 0) || (max_pt.z <= 0))		obstacle_flag = false;
			else obstacle_flag = true;

		}
		else
		{
			obstacle_flag = false;
		}

		pcl::visualization::PointCloudColorHandlerCustom<CloudType> single_color1(cloud_cluster, 0, 255, 0);
		viewer->addPointCloud<CloudType> (cloud_cluster, single_color1, "cloud 1");

		pcl::visualization::PointCloudColorHandlerCustom<CloudType> single_color2(cloud_cluster_closest, 255, 0, 0);
		viewer->addPointCloud<CloudType>  (cloud_cluster_closest, single_color2, "cloud 2" );

		pcl::visualization::PointCloudColorHandlerCustom<CloudType> single_color3(cloud_plane, 0, 0, 255);
		viewer->addPointCloud<CloudType>  (cloud_plane, single_color3, "cloud 3" );	

		viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud 1");
		viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud 2");
		viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud 3");
//		viewer->addCoordinateSystem (1.0);
		viewer->spinOnce();
		viewer->initCameraParameters ();

		return (cloud_cluster);
	}
};

//------------------------------------------------------------------------------------------------------
//Controller thread
//------------------------------------------------------------------------------------------------------
UINT CRobotVisionDlg::ControllerThread(LPVOID pParam) 
{
	CRobotVisionDlg *pContr = (CRobotVisionDlg*) pParam;
	pContr->ControllerThread();

	return 0;
}

void CRobotVisionDlg::ControllerThread()
{

/* 	CvFont font;
	cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX, 1, 1, 10);
	while(1)
	{
		
		
		
		frame = cvQueryFrame(capture);
		if(!frame) break;

		
		if(obstacle_flag)
		{
			std::stringstream ss1, ss2;
		POS:	
			minpointTemp = min_pt;
			maxpointTemp = max_pt;	
			
			if(minpointTemp.z == 0 || maxpointTemp.z == 0)
				{
					Sleep(5);
					goto POS;
				}
			
			escapeObstacle(maxpointTemp, minpointTemp);
			sendCommand(direction, anpha_cmd);
			ss1 << doneRobot << "|";
			while(!doneRobot)
			{
				Sleep(5);
			}
			ss1 << anpha_cmd << "|" << direction << "|" << doneRobot;
			cvPutText( frame, ss1.str().c_str(), cvPoint( 0, 220), &font, cvScalar(0, 0, 255) );
			
			safe_move();
			sendCommand(direction, safe_distance);
			while(!doneRobot)
			{
				Sleep(5);
			}
			
			ss2 << direction << "|" << safe_distance;
			cvPutText( frame, ss2.str().c_str(), cvPoint( 0, 250), &font, cvScalar(255, 0, 0) );

		}
		cvShowImage("Camera", frame);

		char c=cvWaitKey(33);
		if(c==27) break;

	}
	cvReleaseCapture(&capture);
	cvDestroyWindow("Camera");   */
	
LABEL2:
	goTarget();
	sendCommand(direction, anpha_cmd);
	while(!doneRobot)
	{
		Sleep(5);
	}
	updateStatus();
	
	if(obstacle_flag) //1st time
	{
		goto LABEL3;		
	}
	else
	{
		direction = "T";
		sendCommand(direction, distance_to_target);
LABEL1:
		if(obstacle_flag) //2nd time
		{
			if(direction == "T") 
			{
				updateStatus();
			}	

LABEL3:
			minpointTemp = min_pt;
			maxpointTemp = max_pt;	
			
			if(minpointTemp.z == 0 || maxpointTemp.z == 0)
			{
				Sleep(5);
				goto LABEL3;
			}
			
			escapeObstacle(maxpointTemp, minpointTemp);
			sendCommand(direction, anpha_cmd);
LABEL4:		while(!doneRobot)
			{
				Sleep(5);
			}
			updateStatus();
			goto LABEL1;	
		}
		else
		{
			if(direction == "T")
			{
				if(doneRobot)
				{
					//At the target, do nothing
					SetDlgItemText(IDC_EDIT_NUM_AT_TARGET, "Yes");
					ContrThread->SuspendThread();
					//while(1);
				}
				else
				{
					goto LABEL1;	
				}
			}
			else
			{
				if(direction == "F")
				{
					goto LABEL2;
				}
				else
				{
					safe_move();
					sendCommand(direction, safe_distance);
					goto LABEL4;
				}
			}	

		}	
	}
}

void CRobotVisionDlg::KinectThread()
{

	
	// init grabber for MS Kinect:
	local local_;
	std::string device_id("");
	pcl::Grabber* intf = new pcl::OpenNIGrabber (device_id,	pcl::OpenNIGrabber:: OpenNI_QVGA_30Hz,pcl::OpenNIGrabber:: OpenNI_QVGA_30Hz);
	boost::function<void (const CloudConstPtr&)> cloud_cb = boost::bind(&local::cloud_callback,&local_, _1);
	boost::signals2::connection cloud_connection = intf->registerCallback (cloud_cb);
	intf->start ();

	while (!viewer->wasStopped())
	{
		if (local_.cloud_)
		{
			FPS_CALC ("drawing");
			local_.getLatestCloud();
			
			if(obstacle_flag) SetDlgItemText(IDC_EDIT_OBS, "Yes");
			else SetDlgItemText(IDC_EDIT_OBS, "No");

			std::stringstream ss;
			ss << numObs;
			SetDlgItemText(IDC_EDIT_NUM_OBS, ss.str().c_str());
		}
	}

	intf->stop ();
	cloud_connection.disconnect();
}

// CAboutDlg dialog used for App About

class CAboutDlg : public CDialogEx
{
public:
	CAboutDlg();

// Dialog Data
	enum { IDD = IDD_ABOUTBOX };

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support

// Implementation
protected:
	DECLARE_MESSAGE_MAP()
};

CAboutDlg::CAboutDlg() : CDialogEx(CAboutDlg::IDD)
{
}

void CAboutDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
}

BEGIN_MESSAGE_MAP(CAboutDlg, CDialogEx)
END_MESSAGE_MAP()


// CRobotVisionDlg dialog




CRobotVisionDlg::CRobotVisionDlg(CWnd* pParent /*=NULL*/)
	: CDialogEx(CRobotVisionDlg::IDD, pParent)
	, m_strNamePort(_T(""))
	, m_strBaudRate(_T(""))
	, m_strTextSend(_T(""))
	, m_strTextReceive(_T(""))
	, m_strPortStatus(_T(""))
	, openPortActivate(false)
	, closePortActivate(false)
	, sendActivate(false)
	, activeProccess(FALSE)
	, m_strtest(_T(""))
	, m_strXCurrent(_T(""))
	, m_strYCurrent(_T(""))
	, m_strAngleCurrent(_T(""))
	, m_strXTarget(_T(""))
	, m_strYTarget(_T(""))
	, m_dbXCurrent(0)
	, m_dbYCurrent(0)
	, m_dbAngleCurrent(0)
	, m_dbXTarget(0)
	, m_dbYTarget(0)
	, m_angle(0)
	, doneRobot(false)
{
	m_hIcon = AfxGetApp()->LoadIcon(IDR_MAINFRAME);
	m_strNamePort = "COM3";
	m_strBaudRate = "19200";
}

void CRobotVisionDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
	DDX_CBString(pDX, IDC_COMBO_NAME_PORT, m_strNamePort);
	DDX_CBString(pDX, IDC_COMBO_BOUDRATE, m_strBaudRate);
	DDX_Text(pDX, IDC_EDIT_TEXT_SEND, m_strTextSend);
	DDX_Text(pDX, IDC_EDIT_TEXT_RECEIVE, m_strTextReceive);
	DDX_Text(pDX, IDC_EDIT_STATUS_PORT, m_strPortStatus);
	DDX_Text(pDX, IDC_EDIT_X_CURRENT, m_strXCurrent);
	DDX_Text(pDX, IDC_EDIT_Y_CURRENT, m_strYCurrent);
	DDX_Text(pDX, IDC_EDIT_ANGLE_CURRENT, m_strAngleCurrent);
	DDX_Text(pDX, IDC_EDIT_X_TARGET, m_strXTarget);
	DDX_Text(pDX, IDC_EDIT_Y_TARGET, m_strYTarget);
	DDX_Control(pDX, IDC_SLIDER_TILT, m_sliderTilt);
	DDX_Slider(pDX, IDC_SLIDER_TILT, m_angle);
}

BEGIN_MESSAGE_MAP(CRobotVisionDlg, CDialogEx)
	ON_WM_SYSCOMMAND()
	ON_WM_PAINT()
	ON_WM_QUERYDRAGICON()
	ON_BN_CLICKED(IDC_BUTTON_OPEN_PORT, &CRobotVisionDlg::OnBnClickedButtonOpenPort)
	ON_BN_CLICKED(IDC_BUTTON_CLOSE_PORT, &CRobotVisionDlg::OnBnClickedButtonClosePort)
	ON_BN_CLICKED(IDC_BUTTON_EXIT, &CRobotVisionDlg::OnBnClickedButtonExit)
	ON_BN_CLICKED(IDC_BUTTON_SEND_DATA, &CRobotVisionDlg::OnBnClickedButtonSendData)
	ON_BN_CLICKED(IDC_BUTTON_RUN, &CRobotVisionDlg::OnBnClickedButtonRun)
	ON_NOTIFY(NM_CUSTOMDRAW, IDC_SLIDER_TILT, &CRobotVisionDlg::OnNMCustomdrawSliderTilt)
	ON_BN_CLICKED(IDC_BUTTON1, &CRobotVisionDlg::OnBnClickedButton1)
END_MESSAGE_MAP()


// CRobotVisionDlg message handlers

BOOL CRobotVisionDlg::OnInitDialog()
{
	CDialogEx::OnInitDialog();

	// Add "About..." menu item to system menu.

	// IDM_ABOUTBOX must be in the system command range.
	ASSERT((IDM_ABOUTBOX & 0xFFF0) == IDM_ABOUTBOX);
	ASSERT(IDM_ABOUTBOX < 0xF000);

	CMenu* pSysMenu = GetSystemMenu(FALSE);
	if (pSysMenu != NULL)
	{
		BOOL bNameValid;
		CString strAboutMenu;
		bNameValid = strAboutMenu.LoadString(IDS_ABOUTBOX);
		ASSERT(bNameValid);
		if (!strAboutMenu.IsEmpty())
		{
			pSysMenu->AppendMenu(MF_SEPARATOR);
			pSysMenu->AppendMenu(MF_STRING, IDM_ABOUTBOX, strAboutMenu);
		}
	}

	// Set the icon for this dialog.  The framework does this automatically
	//  when the application's main window is not a dialog
	SetIcon(m_hIcon, TRUE);			// Set big icon
	SetIcon(m_hIcon, FALSE);		// Set small icon

	// TODO: Add extra initialization here
	GetDlgItem(IDC_BUTTON_CLOSE_PORT)->EnableWindow(FALSE);
	serialProcess = (SerialThread*)AfxBeginThread(RUNTIME_CLASS(SerialThread), 
		THREAD_PRIORITY_NORMAL, 0, CREATE_SUSPENDED);
	serialProcess->setOwner(this);

	m_sliderTilt.SetRange(CL_MOTOR_MIN_ANGLE, CL_MOTOR_MAX_ANGLE);
	m_sliderTilt.SetTicFreq(1000.0);
	m_sliderTilt.SetPos(CL_MOTOR_MIN_ANGLE);
	initMotorControl();

	return TRUE;  // return TRUE  unless you set the focus to a control
}

void CRobotVisionDlg::OnSysCommand(UINT nID, LPARAM lParam)
{
	if ((nID & 0xFFF0) == IDM_ABOUTBOX)
	{
		CAboutDlg dlgAbout;
		dlgAbout.DoModal();
	}
	else
	{
		CDialogEx::OnSysCommand(nID, lParam);
	}
}

// If you add a minimize button to your dialog, you will need the code below
//  to draw the icon.  For MFC applications using the document/view model,
//  this is automatically done for you by the framework.

void CRobotVisionDlg::OnPaint()
{
	if (IsIconic())
	{
		CPaintDC dc(this); // device context for painting

		SendMessage(WM_ICONERASEBKGND, reinterpret_cast<WPARAM>(dc.GetSafeHdc()), 0);

		// Center icon in client rectangle
		int cxIcon = GetSystemMetrics(SM_CXICON);
		int cyIcon = GetSystemMetrics(SM_CYICON);
		CRect rect;
		GetClientRect(&rect);
		int x = (rect.Width() - cxIcon + 1) / 2;
		int y = (rect.Height() - cyIcon + 1) / 2;

		// Draw the icon
		dc.DrawIcon(x, y, m_hIcon);
	}
	else
	{
		CDialogEx::OnPaint();
	}
}

// The system calls this function to obtain the cursor to display while the user drags
//  the minimized window.
HCURSOR CRobotVisionDlg::OnQueryDragIcon()
{
	return static_cast<HCURSOR>(m_hIcon);
}



void CRobotVisionDlg::UpdateConfig(void)
{
	// constant parameter.
	configSerial_.ByteSize = 8;
	configSerial_.StopBits = ONESTOPBIT;
	configSerial_.Parity = NOPARITY;

	switch(atoi(m_strBaudRate))
	{
	case 110:
		configSerial_.BaudRate = CBR_110;
		break;
	case 300:
		configSerial_.BaudRate = CBR_300;
		break;
	case 600:
		configSerial_.BaudRate = CBR_600;
		break;
	case 1200:
		configSerial_.BaudRate = CBR_1200;
		break;
	case 2400:
		configSerial_.BaudRate = CBR_2400;
		break;
	case 4800:
		configSerial_.BaudRate = CBR_4800;
		break;
	case 9600:
		configSerial_.BaudRate = CBR_9600;
		break;
	case 14400:
		configSerial_.BaudRate = CBR_14400;
		break;
	case 19200:
		configSerial_.BaudRate = CBR_19200;
		break;
	case 38400:
		configSerial_.BaudRate = CBR_38400;
		break;
	case 56000:
		configSerial_.BaudRate = CBR_56000;
		break;
	case 57600:
		configSerial_.BaudRate = CBR_57600;
		break;
	case 115200 :
		configSerial_.BaudRate = CBR_115200;
		break;
	case 128000:
		configSerial_.BaudRate = CBR_128000;
		break;
	case 256000:
		configSerial_.BaudRate = CBR_256000;
		break;
	default:
		break;
	}
}


void CRobotVisionDlg::OnBnClickedButtonOpenPort()
{
	// TODO: Add your control notification handler code here
	UpdateData(TRUE);
	UpdateConfig();
	openPortActivate = true;
	closePortActivate = false;
	activeProccess = TRUE;
	UpdateData(TRUE);
	serialProcess->ResumeThread();
	GetDlgItem(IDC_BUTTON_CLOSE_PORT)->EnableWindow(TRUE);
	GetDlgItem(IDC_BUTTON_OPEN_PORT)->EnableWindow(FALSE);
	UpdateData(FALSE);
}


void CRobotVisionDlg::OnBnClickedButtonClosePort()
{
	// TODO: Add your control notification handler code here
	// Set signal of closing port serial communication.
	closePortActivate =  true;
	openPortActivate = false;
	GetDlgItem(IDC_BUTTON_CLOSE_PORT)->EnableWindow(FALSE);
	GetDlgItem(IDC_BUTTON_OPEN_PORT)->EnableWindow(TRUE);
	UpdateData(FALSE);
}


void CRobotVisionDlg::OnBnClickedButtonExit()
{
	// TODO: Add your control notification handler code here
	// Set signal of closing port serial communication.
	stopMotorControl();
	serialProcess->SuspendThread();
	this->DestroyWindow();
}


void CRobotVisionDlg::OnBnClickedButtonSendData()
{
	// TODO: Add your control notification handler code here
	// Set signal to send data of serial communication.
	UpdateData(TRUE);
	sendActivate = true;
}


void CRobotVisionDlg::OnBnClickedButtonRun()
{
	UpdateData(TRUE);
	m_sliderTilt.SetPos(CL_MOTOR_MIN_ANGLE);
	Sleep(500);

	pass_.setFilterFieldName ("z");
	pass_.setFilterLimits (0.5, 1.5);

	//	grid_.setFilterFieldName ("z");
	//	grid_.setFilterLimits (0.5, 1.0);
	grid_.setLeafSize (0.04, 0.04, 0.04);

	seg_.setOptimizeCoefficients (true);
	seg_.setModelType (pcl::SACMODEL_PLANE);
	seg_.setMethodType (pcl::SAC_RANSAC);
	seg_.setMaxIterations (2000);
	seg_.setDistanceThreshold (0.02); //2cm
	//	extract_.setNegative (true);

	ec.setClusterTolerance (0.05); // 3cm
	ec.setMinClusterSize (20);
	ec.setMaxClusterSize (1000);
	ec.setSearchMethod (tree);
	
	KinectThread();
}


void CRobotVisionDlg::OnNMCustomdrawSliderTilt(NMHDR *pNMHDR, LRESULT *pResult)
{
	LPNMCUSTOMDRAW pNMCD = reinterpret_cast<LPNMCUSTOMDRAW>(pNMHDR);
	// TODO: Add your control notification handler code here
	m_angle = m_sliderTilt.GetPos();
	setMotorAngle(m_angle);

	*pResult = 0;
}


void CRobotVisionDlg::OnBnClickedButton1()
{
	// TODO: Add your control notification handler code here
/* 	capture = cvCreateCameraCapture(0);
	cvNamedWindow("Camera", CV_WINDOW_AUTOSIZE); */
	
	//-----------------------------
	//Controller thread Initiation
	//-----------------------------
		m_dbXcurrent_Temp = 0;
		m_dbXCurrent = 0;
		m_dbYcurrent_Temp = 0;
		m_dbYCurrent = 0;
		m_dbAngleCurrent_Temp = 0;
		m_dbAngleCurrent = 90.0;
		direction_old = "";
		SetDlgItemText(IDC_EDIT_NUM_AT_TARGET, "No");
		UpdateData(TRUE);
	//	updateStatus();
		m_dbXTarget = atof(m_strXTarget);
		m_dbYTarget = atof(m_strYTarget);
	if(bFirst)
	{
		ContrThread = AfxBeginThread(ControllerThread, this);
		bFirst = false;
	}
	else ContrThread->ResumeThread();	

	
}
//----------------------------------------------------------------------------------------
//Functions for controller process
//----------------------------------------------------------------------------------------
//compute safe angle rotation and distance to target
void CRobotVisionDlg::goTarget(void)
{
	anpha = atan((m_dbXTarget - m_dbXcurrent_Temp)/(m_dbYTarget - m_dbYcurrent_Temp)); //assume Y is always  positive
	if(anpha < 0) 
	{
		anpha = -anpha;
		direction = "L";
	}
	else direction = "R";
	
	anpha_cmd = anpha*180.0f/PI;
	
	distance_to_target = sqrt((m_dbXTarget - m_dbXcurrent_Temp)*(m_dbXTarget - m_dbXcurrent_Temp)
						 + (m_dbYTarget - m_dbYcurrent_Temp)*(m_dbYTarget - m_dbYcurrent_Temp));
}
//----------------------------------------------------------------------------------------
//Update robot's status (X_cur, Y_cur, Angle_cur, ...)
void CRobotVisionDlg::updateStatus()
{
	m_dbXcurrent_Temp = m_dbXCurrent; 
	m_dbYcurrent_Temp = m_dbYCurrent;
	m_dbAngleCurrent_Temp = m_dbAngleCurrent;
}

//----------------------------------------------------------------------------------------
//Send command to micro-controller through RS232 interface
void CRobotVisionDlg::sendCommand(CString dir, double val)
{
	UpdateData(TRUE);
	std::stringstream ss;
	size_t decimal_places(3); //need 3 digits decimal

	ss << "*" << dir << setprecision( static_cast< int >( log10( val ) ) + 1 + decimal_places ) << val << "#";
	m_strTextSend = ss.str().c_str();
	sendActivate = true;
	doneRobot = false;
	UpdateData(FALSE);
}
//----------------------------------------------------------------------------------------
//Calculate angle for the robot's safe rotation
void CRobotVisionDlg::escapeObstacle(CloudType& max_point, CloudType& min_point)
{
	if(max_point.x < 0 && max_point.x > - RobotWidth/2.0f) 	
	{
		anpha = acos(- max_point.x/RobotCross) - acos(RobotWidth/RobotCross/2.0f);
		direction = "R";
	}	
	else if (min_point.x > 0 && min_point.x < RobotWidth/2.0f) 
	{
		anpha = acos(min_point.x/RobotCross) - acos(RobotWidth/RobotCross/2.0f);
		direction = "L";
	}
	else if (max_point.x > 0 && min_point.x < 0)
	{
		if(max_point.x < - min_point.x)	
		{
			anpha = asin(RobotWidth/RobotCross/2.0f) + asin(max_point.x/RobotCross);
			direction = "R";
		}	
		else 
		{
			anpha = asin(RobotWidth/RobotCross/2.0f) + asin(- min_point.x/RobotCross);
			direction = "L";
		}
	}
	else
	{
		anpha = 0;
		direction = "R";
	}

	anpha_cmd =  anpha*180.0f/PI;
	//safe_distance = angle_current 	
}
//----------------------------------------------------------------------------------------
//safe distance for robot avoid obstacle completely
void CRobotVisionDlg::safe_move(void)
{
	double distanceTemp = 0;
	//	updateStatus();
	while(!distanceTemp)
	{
		if(direction == "R" || direction == "L")
		{
			distanceTemp = sqrt(maxpointTemp.z*maxpointTemp.z + maxpointTemp.x*maxpointTemp.x)*cos(atan(abs(maxpointTemp.x)/maxpointTemp.z) + anpha) + 0.15;
		}
	}	
//	assert(distanceTemp != 0);
/* 	else if(direction == "L")
	{
		distanceTemp = sqrt(minpointTemp.z*minpointTemp.z + minpointTemp.x*minpointTemp.x)*cos(atan(abs(minpointTemp.x)/minpointTemp.z) + anpha) + ;
	} */
	direction = "F";
	safe_distance = 100.0f*distanceTemp;
}