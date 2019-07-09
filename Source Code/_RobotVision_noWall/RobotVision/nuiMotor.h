//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// This library is part of CL NUI SDK
// It allows the use of Microsoft Kinect cameras in your own applications
//
// For updates and file downloads go to: http://codelaboratories.com/get/kinect
//
// Copyright 2010 (c) Code Laboratories, Inc.  All rights reserved.
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#ifndef NUIMOTOR_H
#define NUIMOTOR_H

#pragma once
#include <windows.h>
#include <CLNUIDevice.h>
//#pragma comment(lib, "CLNUIDevice.lib")

#define IMPORT(type) extern "C" __declspec(dllimport)## type __cdecl

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// NUIDevice  API
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Device enumeration
IMPORT(int) GetNUIDeviceCount();
IMPORT(PCHAR) GetNUIDeviceSerial(int index);

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// CLNUIMotor  API
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Motor instance type
typedef void *CLNUIMotor;

// Library initialization
IMPORT(CLNUIMotor) CreateNUIMotor(PCHAR serial);
IMPORT(bool) DestroyNUIMotor(CLNUIMotor mot);

// Motor control
IMPORT(bool) SetNUIMotorPosition(CLNUIMotor mot, SHORT position);

// Get accelerometer data
IMPORT(bool) GetNUIMotorAccelerometer(CLNUIMotor mot, SHORT &x, SHORT &y, SHORT &z);

IMPORT(bool) SetNUIMotorLED(CLNUIMotor mot, BYTE value);

#define CL_MOTOR_MAX_ANGLE 9000
#define CL_MOTOR_MIN_ANGLE -16000

void initMotorControl();
void stopMotorControl();
void setMotorAngle(int angle);
void resetMotorAngle();

#endif