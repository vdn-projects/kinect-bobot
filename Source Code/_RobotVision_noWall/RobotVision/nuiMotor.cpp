#include "stdafx.h"
#include "nuiMotor.h"

PCHAR deviceSerial;
CLNUIMotor motor;
int motorAngle;

void initMotorControl()
{
        deviceSerial = GetNUIDeviceSerial(0);
        motor = CreateNUIMotor (deviceSerial);
   //     motorAngle = 0;
   //     SetNUIMotorPosition(motor, motorAngle);
}

void stopMotorControl()
{
        DestroyNUIMotor (motor);
}

void setMotorAngle(int angle)
{
     //   motorAngle+= angle;
	if(angle <= CL_MOTOR_MAX_ANGLE && angle >= CL_MOTOR_MIN_ANGLE)
        SetNUIMotorPosition(motor, angle);
	else return;
}

void resetMotorAngle()
{
        motorAngle = 0;
        SetNUIMotorPosition(motor, motorAngle);
}