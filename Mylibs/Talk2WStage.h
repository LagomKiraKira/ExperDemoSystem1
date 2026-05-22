/*
* Talk2WStage.h
*
* Created on SEP 2023
* by Shuhao Yang
*
*/

#pragma once

#ifndef TALK2WSTAGE_H_
#define TALK2WSTAGE_H_

#pragma comment(lib, "SLS_Dll.lib")

#include <iostream>
#include "SLS_Dll.h"
#include <cstring>
#include <cstdlib>
#include <windows.h>
#include <opencv/highgui.h>

#define AXIS_X 255
#define AXIS_Y 254
#define STAGERROR FALSE
#define STAGESUCC TRUE
#define COM 3    //运行前查看最新的位移台端口号->需要相应修改
#define SPACE 1

typedef struct StageDataStruct {
	int handle;

	double AxisPos_x[1];
	double AxisPos_y[1];
	double prePos_x;
	double prePos_y;
	CvPoint stageVel;   //Current velocity of stage
	float smoothed_vel_x = 0.0f;
	float smoothed_vel_y = 0.0f;
	CvPoint stageCenter;   // Point indicating center of stage.
	CvPoint stageFeedbackTargetOffset;   //Target of the stage feedback loop as a delta distance in pixels from the center of the image
	int stageIsTurningOff;   //1 indicates stage is turning off. 0 indicates stage is on or off

	// PID constants (these values need tuning)
	float Kp_x = 6, Ki_x = 0, Kd_x = 0.45;  // PID coefficients for X axis
	float Kp_y = 6, Ki_y = 0, Kd_y = 0.45;  // PID coefficients for Y axis

	// PID variables for both axes
	float previous_error_x = 0.0, integral_x = 0.0;
	float previous_error_y = 0.0, integral_y = 0.0;

	// Target time to reach the target (in ms), passed as parameter t
	float target_time = 80.0; // Example: 1 second to move (t)


	bool stageIsPresent;
}StageData;

/* Allocate Memory for StageData structure. */
bool AllocateStageDataStructMemory(StageData** Stage);

/* Initialize USB Stage COM and get the Stage handle */
bool Initialize_StageCOMAndAxis(StageData& Stage);

void UnInitialize_Stage(StageData& Stage);

bool FindStagePosition(StageData& Stage);

void HaltStage(StageData& Stage);

void steerStageFromNumberPad(StageData& Stage, int input);

void MoveToCenter(StageData& Stage, float Center_X, float Center_Y);

void T2Stage_TurnOff(StageData** Stage);

void spinStage(StageData& Stage, float vel_x, float vel_y);

void MoveToTarget(StageData& Stage, int pixel_x, int pixel_y);

void MoveXToTarget(StageData& Stage, int pixel_x);

void MoveYToTarget(StageData& Stage, int pixel_y);

#endif