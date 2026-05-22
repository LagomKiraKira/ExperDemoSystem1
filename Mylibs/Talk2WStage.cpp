#include "Talk2WStage.h"

/* Allocate Memory for StageData structure. */
bool AllocateStageDataStructMemory(StageData** Stage)
{
	*Stage = new StageData;
	if (*Stage == NULL)
	{
		std::cout << "Can't allocate memory for StageDataStruct." << std::endl;
		return STAGERROR;
	}

	std::cout << "Allocate memory for StageDataStruct." << std::endl;
	return STAGESUCC;
}

/* Initialize USB Stage COM and get the Stage handle */
bool Initialize_StageCOMAndAxis(StageData& Stage)
{
	Stage.handle = InitCom(COM);
	if (Stage.handle != 1)
	{
		std::cout << "Can't Initialize the Stage COM\n";
		return STAGERROR;
	}
	else
		std::cout << "Initialize the Stage COM " << COM << std::endl;

	if (!InitStage(Stage.handle, AXIS_X, new float[3]{ 10000, 10, 0.01f })
		|| !InitStage(Stage.handle, AXIS_Y, new float[3]{ 10000, 10, 0.01f }))
	{
		std::cout << "Can't Initialize Axis X or Y\n";
		return STAGERROR;
	}

	//设置轴的速度和加速度
	SpeedSet(Stage.handle, 20, AXIS_X);
	SpeedSet(Stage.handle, 20, AXIS_Y);
	AcDecSet(Stage.handle, 100, AXIS_X);
	AcDecSet(Stage.handle, 100, AXIS_Y);

	//初始化各种参数
	Stage.AxisPos_x[0] = 0;
	Stage.AxisPos_y[0] = 0;
	Stage.prePos_x = 0;
	Stage.prePos_y = 0;
	Stage.stageVel = cvPoint(0, 0);
	Stage.stageCenter = cvPoint(0, 0);
	Stage.stageFeedbackTargetOffset = cvPoint(0, 0);
	Stage.stageIsTurningOff = 0;

	Stage.stageIsPresent = FALSE;

	return STAGESUCC;
}

void UnInitialize_Stage(StageData& Stage)
{
	UnInitStage(Stage.handle, AXIS_X);
	UnInitStage(Stage.handle, AXIS_Y);

	UnInitCom(COM);
}

/* 获取当前位移台的位置参数 */
bool FindStagePosition(StageData& Stage)
{
	if (!GetPosition(Stage.handle, AXIS_X, Stage.AxisPos_x) ||
		!GetPosition(Stage.handle, AXIS_Y, Stage.AxisPos_y))
	{
		std::cout << "Can't Get the X Y position" << std::endl;
		return STAGERROR;
	}

	return STAGESUCC;
}

void HaltStage(StageData& Stage)
{
	Stop(Stage.handle, AXIS_X);
	Stop(Stage.handle, AXIS_Y);
}

void steerStageFromNumberPad(StageData& Stage, int input)
{
	switch (input) {
		/** Cardinal Directions **/
	case 6:
		printf("Right!\n");
		MoveRelativeWait(Stage.handle, SPACE, AXIS_X);
		break;
	case 8:
		printf("Up!\n");
		MoveRelativeWait(Stage.handle, SPACE, AXIS_Y);
		break;
	case 4:
		printf("Left!\n");
		MoveRelativeWait(Stage.handle, 0 - SPACE, AXIS_X);
		break;
	case 2:
		printf("Down!\n");
		MoveRelativeWait(Stage.handle, 0 - SPACE, AXIS_Y);
		break;
		/** Multiples of 45 **/
	case 9:
		printf("Up-Right!\n");
		MoveRelativeWait(Stage.handle, SPACE, AXIS_X);
		MoveRelativeWait(Stage.handle, SPACE, AXIS_Y);
		break;
	case 7:
		printf("Up-Left!\n");
		MoveRelativeWait(Stage.handle, SPACE, AXIS_Y);
		MoveRelativeWait(Stage.handle, 0 - SPACE, AXIS_X);
		break;
	case 1:
		printf("Down-Left!\n");
		MoveRelativeWait(Stage.handle, 0 - SPACE, AXIS_X);
		MoveRelativeWait(Stage.handle, 0 - SPACE, AXIS_Y);
		break;
	case 3:
		printf("Down-Right!\n");
		MoveRelativeWait(Stage.handle, 0 - SPACE, AXIS_Y);
		MoveRelativeWait(Stage.handle, SPACE, AXIS_X);
		break;
	case 5:
		printf("HALT!\n");
		HaltStage(Stage);
		break;
	default:
		break;
	}
	return;
}

void MoveToCenter(StageData& Stage, float Center_X, float Center_Y)
{
	HomeNew(Stage.handle, AXIS_X);
	bool a[1];
	Sleep(200);
	while (true)
	{
		bool res = GetStation(Stage.handle, AXIS_X, a);
		if (res)
		{
			if (!a[0])
				break;
		}
		Sleep(1);
	}
	HomeNew(Stage.handle, AXIS_Y);
	bool b[1];
	Sleep(200);
	while (true)
	{
		bool res = GetStation(Stage.handle, AXIS_Y, b);
		if (res)
		{
			if (!b[0])
				break;
		}
		Sleep(1);
	}

	if (FindStagePosition(Stage))
		std::cout << "x:" << Stage.AxisPos_x[0] << ",y:" << Stage.AxisPos_y[0] << std::endl;

	MoveRelativeWait(Stage.handle, 0 - 60, AXIS_X);
	MoveRelativeWait(Stage.handle, 0 - 35, AXIS_Y);

	if (FindStagePosition(Stage))
		std::cout << "x:" << Stage.AxisPos_x[0] << ",y:" << Stage.AxisPos_y[0] << std::endl;

	SetPosition(Stage.handle, 0, AXIS_X);
	SetPosition(Stage.handle, 0, AXIS_Y);

	if (FindStagePosition(Stage))
		std::cout << "Reset:  x:" << Stage.AxisPos_x[0] << ",y:" << Stage.AxisPos_y[0] << std::endl;

	MoveRelativeWait(Stage.handle, Center_X, AXIS_X);
	MoveRelativeWait(Stage.handle, Center_Y, AXIS_Y);

	if (FindStagePosition(Stage))
		std::cout << "Center:  x:" << Stage.AxisPos_x[0] << ",y:" << Stage.AxisPos_y[0] << std::endl;

	SetPosition(Stage.handle, 0, AXIS_X);
	SetPosition(Stage.handle, 0, AXIS_Y);

	if (FindStagePosition(Stage))
		std::cout << "Reset:  x:" << Stage.AxisPos_x[0] << ",y:" << Stage.AxisPos_y[0] << std::endl;

	//设定位移台准备状态
	Stage.stageIsPresent = TRUE;
}

void T2Stage_TurnOff(StageData** Stage)
{
	if (*Stage != NULL)
	{
		free(*Stage);
		*Stage = NULL;
	}
	return;
}

void spinStage(StageData& Stage, float vel_x, float vel_y)
{
	//设置轴的速度
	SpeedSet(Stage.handle, vel_x, AXIS_X);
	SpeedSet(Stage.handle, vel_y, AXIS_Y);

	return;
}


void MoveToTarget(StageData& Stage, int pixel_x, int pixel_y)//单位pixel/s
{
	float resolution = static_cast<float>(0.1 / 108); 

	MoveRelative(Stage.handle, pixel_x * resolution, AXIS_X);
	MoveRelative(Stage.handle, pixel_y * resolution, AXIS_Y);
}

void MoveXToTarget(StageData& Stage, int pixel_x) 
{
	float resolution = static_cast<float>(0.1 / 108);

	MoveRelative(Stage.handle, pixel_x * resolution, AXIS_X);
}

void MoveYToTarget(StageData& Stage, int pixel_y)
{
	float resolution = static_cast<float>(0.1 / 108);

	MoveRelative(Stage.handle, pixel_y * resolution, AXIS_Y);
}