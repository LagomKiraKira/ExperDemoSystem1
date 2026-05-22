/*
* Talk2MomentCamera.h
*
* Created on SEP 2023
* by Shuhao Yang
*
*/

#pragma once

#include "Common.h"

#define CAMERROR FALSE
#define CAMSUCC TRUE
#define WIDTH 1600
#define HEIGHT 1100

typedef struct CamDataStruct {
	CameraContext* ctx;
	std::vector<CameraContext*> contexts;
	unsigned char* iImageData;
	uint64_t  iFrameNumber;

	uns32 exposureBytes;   //曝光字节数
	uns32 exposureTime;    //曝光时间
	uns16 circBufferFrames;    //Buffer覆盖循环次数
	int16 bufferMode;    //曝光模式
	uns32 circBufferBytes;   //Buffer字节数
	uns8* circBufferInMemory;  //循环Buffer内存

	/* Find MinValue and MaxValue in the first Frame */
	uns16 minValue;   
	uns16 maxValue;
	bool errorOccurred;
	bool resetMaxAndMin;
}CamData;

/* Check Pvcam Information before operations on Camera */
bool T2Cam_ShowPvcamInfo(int argc, char* argv[]);

/* Allocate Memory for CamData structure. */
bool T2Cam_AllocateCamDataStructMemory(CamData** Camera);

/* Connect to the camera and open Specific Camera*/
bool T2Cam_InitAndOpenSpecificCamera(CamData& Camera);

/* Start grabbing frame. */
bool T2Cam_GrabFramesAsFastAsYouCan(CamData& Camera);

/* Retrieve the Grab Result from camera to CamData structure.
 * Note: this function needs to be called every time you want a new frame
 * to do further analysis. */
bool T2Cam_RetrieveAnImage(CamData& Camera);

/* Turn the camera off and deallocate CamData structure and iPylon decive. */
void T2Cam_TurnOff(CamData& Camera);